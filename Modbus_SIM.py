#!/usr/bin/env python3
"""
MODBUS PLC Simulator  -  TCP/IP + RTU serial slave with a Tkinter GUI.

Requires:  Python >= 3.10, pymodbus >= 3.13 (SimData / SimDevice API), pyserial
           pip install -U pymodbus pyserial

Design notes
------------
* Back-end (`ModbusSimulatorBackend`) runs pymodbus in its own thread/event loop.
  The back-end keeps a plain-Python copy of the four data tables (`model`), which
  is what the GUI displays.  A pymodbus `SimDevice.action` hook mirrors every
  master write into that copy and posts an event on a thread-safe queue.
* The GUI never touches Tk from the server thread: it drains the queue with
  `root.after()` in the main thread.
"""
from __future__ import annotations

import asyncio
import json
import queue
import re
import socket
import sys
import threading
import traceback
import tkinter as tk
import tkinter.font as tkfont
from tkinter import filedialog, messagebox, ttk
from typing import NamedTuple


def _fatal(message: str) -> None:
    """Report a start-up problem and keep a double-click console open long enough to read it."""
    print(message, file=sys.stderr)
    try:
        input("Press Enter to close...")
    except EOFError:
        pass
    sys.exit(1)


if sys.version_info < (3, 10):  # pragma: no cover
    _fatal("Python 3.10 or newer is required.")

try:
    from pymodbus import FramerType, ModbusDeviceIdentification
    from pymodbus.constants import ExcCodes
    from pymodbus.server import ModbusSerialServer, ModbusTcpServer
    from pymodbus.simulator import DataType, SimData, SimDevice
except ImportError as _exc:  # pragma: no cover
    _fatal(f"Cannot import the pymodbus simulator API ({_exc}).\n"
           "Install / upgrade with:   pip install -U pymodbus pyserial")

try:                                     # 3.12 imports fine but rejects separate tables + action
    from importlib.metadata import version as _pkg_version
    _pm = tuple(int(p) for p in _pkg_version("pymodbus").split(".")[:2])
    if _pm < (3, 13):
        _fatal(f"pymodbus {_pkg_version('pymodbus')} found, but 3.13 or newer is required.\n"
               "Upgrade with:   pip install -U pymodbus pyserial")
except (ImportError, ValueError):        # unknown version string -> just try to run
    pass

# ==========================================================================
# Constants
# ==========================================================================
TABLE_SIZE = 65536                      # every Modbus table: addresses 0..65535
COLS = 10                               # registers per grid row (+0 .. +9)
ROWS = (TABLE_SIZE + COLS - 1) // COLS
STRIP_BITS = 52                         # coil quick-view at the bottom
POLL_MS = 50                            # GUI <- back-end event polling


class Table(NamedTuple):
    label: str        # text in the I/O combo box
    prefix: str       # classic reference-number prefix (4xxxx, 0xxxx ...)
    is_bit: bool
    local_fc: int     # function code used when the GUI writes into a running server


TABLES: dict[str, Table] = {
    "hr": Table("Holding Registers (40000)", "4", False, 16),
    "co": Table("Coil Outputs (00000)", "0", True, 15),
    "di": Table("Digital Inputs (10000)", "1", True, 2),
    "ir": Table("Input Registers (30000)", "3", False, 4),
}
LABEL_TO_KIND = {t.label: k for k, t in TABLES.items()}

# Modbus function code -> table
FC_TO_KIND = {1: "co", 5: "co", 15: "co", 2: "di", 3: "hr", 6: "hr",
              16: "hr", 22: "hr", 23: "hr", 4: "ir"}


def reference_number(kind: str, address: int) -> str:
    """0-based protocol address -> classic reference number (40001, 400001 ...)."""
    r = address + 1
    return f"{TABLES[kind].prefix}{r:04d}" if r <= 9999 else f"{TABLES[kind].prefix}{r:05d}"


def natural_key(text: str):
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r"(\d+)", text)]


# ==========================================================================
# 1. BACK-END
# ==========================================================================
class ModbusSimulatorBackend:
    """Modbus slave running in a background thread.

    Events put on `self.events` (consumed by the GUI thread):
        ("started", label)          server is listening
        ("stopped",)                server thread ended
        ("error", text)             start-up / run-time failure
        ("clients", n)              TCP client count changed
        ("write", kind, addr, n)    a master (or local edit) changed n values
    """

    def __init__(self) -> None:
        self.events: queue.Queue = queue.Queue()
        self.model: dict[str, list] = {
            "hr": [0] * TABLE_SIZE, "ir": [0] * TABLE_SIZE,
            "co": [False] * TABLE_SIZE, "di": [False] * TABLE_SIZE,
        }
        self.rx_count = 0
        self.tx_count = 0
        self.clients = 0
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop_evt: asyncio.Event | None = None
        self._server = None
        self._seeding = False

    # ---------------------------------------------------------------- life-cycle
    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start_tcp(self, host: str, port: int) -> None:
        def factory(device):
            return ModbusTcpServer(
                device, framer=FramerType.SOCKET, identity=self._identity(),
                address=(host, port),
                trace_pdu=self._trace_pdu, trace_connect=self._trace_connect)
        self._start(factory, f"TCP {host}:{port}", tcp=True)

    def start_serial(self, port: str, baudrate: int, bytesize: int = 8,
                     parity: str = "N", stopbits: int = 1) -> None:
        def factory(device):
            return ModbusSerialServer(
                device, framer=FramerType.RTU, identity=self._identity(),
                port=port, baudrate=baudrate, bytesize=bytesize,
                parity=parity, stopbits=stopbits, trace_pdu=self._trace_pdu)
        self._start(factory, f"{port} @ {baudrate} {bytesize}{parity}{stopbits}", tcp=False)

    def stop(self) -> None:
        thread, loop, evt = self._thread, self._loop, self._stop_evt
        if thread is not None and thread.is_alive() and loop is not None and evt is not None:
            try:
                loop.call_soon_threadsafe(evt.set)
            except RuntimeError:          # loop already closed
                pass
            thread.join(timeout=3)
        self._thread = self._loop = self._stop_evt = None

    # ---------------------------------------------------------------- internals
    @staticmethod
    def _identity() -> ModbusDeviceIdentification:
        return ModbusDeviceIdentification(info_name={
            "VendorName": "mod_rssim_clone", "ProductName": "Modbus Simulator"})

    def _start(self, factory, label: str, tcp: bool) -> None:
        self.stop()                                   # restart-safe
        self.rx_count = self.tx_count = self.clients = 0
        loop = asyncio.new_event_loop()
        stop_evt = asyncio.Event()                    # loop-agnostic until first await (py>=3.10)
        self._loop, self._stop_evt = loop, stop_evt
        self._thread = threading.Thread(
            target=self._thread_main, args=(loop, stop_evt, factory, label, tcp),
            name="modbus-server", daemon=True)
        self._thread.start()

    def _thread_main(self, loop, stop_evt, factory, label, tcp) -> None:
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._serve(stop_evt, factory, label))
        except Exception as exc:                      # noqa: BLE001 - report everything to the GUI
            self.events.put(("error", self._explain(exc, label, tcp)))
        finally:
            self._server = None
            try:
                loop.run_until_complete(loop.shutdown_asyncgens())
            except Exception:                         # noqa: BLE001
                pass
            loop.close()
            self.events.put(("stopped",))

    async def _serve(self, stop_evt: asyncio.Event, factory, label: str) -> None:
        server = factory(self._build_device())
        try:
            await self._seed(server)                      # before listening: no master can race it
            await server.serve_forever(background=True)   # returns once listening
            self._server = server
            self.events.put(("started", label))
            await stop_evt.wait()
        finally:
            self._server = None
            try:
                await server.shutdown()                   # closes listener + client sockets
            except Exception:                             # noqa: BLE001
                pass

    @staticmethod
    def _explain(exc: Exception, label: str, tcp: bool) -> str:
        text = f"{type(exc).__name__}: {exc}"
        if tcp:
            text += ("\n\nThe server could not listen on " + label.split(" ", 1)[1] + ".\n"
                     "- the port may already be used by another program\n"
                     "- the IP must be one of this PC's addresses (or 0.0.0.0)\n"
                     "- Linux/macOS: ports below 1024 (e.g. 502) need administrator rights")
        else:
            text += ("\n\nCould not open the serial port. Check that it exists and is "
                     "not in use by another program.")
        return text

    # ---------------------------------------------------------------- data model
    @staticmethod
    def _norm(kind: str, values) -> list:
        if TABLES[kind].is_bit:
            return [bool(v) for v in values]
        return [int(v) & 0xFFFF for v in values]

    def _build_device(self) -> SimDevice:
        """Four independent, zero-initialised tables (coils, discrete inputs, holding, input).

        Initial contents are NOT passed in here: on pymodbus 3.13/3.14 SimData(REGISTERS)
        packs signed 16-bit and rejects values above 32767.  `_seed()` copies the model
        into the datastore instead, which accepts the full 0..65535 range.
        """
        # explicit value lists with the default count=1 (count=TABLE_SIZE would multiply
        # the list internally and exhaust memory)
        def bits(): return [SimData(0, values=[False] * TABLE_SIZE, datatype=DataType.BITS)]
        def regs(): return [SimData(0, values=[0] * TABLE_SIZE, datatype=DataType.REGISTERS)]
        return SimDevice(
            id=0,                                     # id 0 = answer to ANY unit id
            action=self._action,
            simdata=(bits(), bits(), regs(), regs()))

    async def _seed(self, server) -> None:
        """Load the model into a freshly built (not yet listening) server."""
        self._seeding = True
        try:
            for kind, table in TABLES.items():
                data = self.model[kind]
                if any(data):
                    await server.async_setValues(0, table.local_fc, 0, list(data))
        finally:
            self._seeding = False

    async def _action(self, function_code, start_address, address, count,
                      registers, values):
        """pymodbus hook: called for every access. `values` is None on reads."""
        if values is None or self._seeding:
            return None
        kind = FC_TO_KIND.get(function_code)
        if kind is None:
            return None
        n = len(values)          # NB: `count` is the register count, not the bit count
        if address < 0 or address + n > TABLE_SIZE:
            # pymodbus rejects such a write *after* this hook, so refuse here and
            # do not pollute the mirror with data that is never stored.
            return ExcCodes.ILLEGAL_ADDRESS
        self.model[kind][address:address + n] = self._norm(kind, values)
        self.events.put(("write", kind, address, n))
        return None

    def _trace_pdu(self, sending: bool, pdu):
        if sending:
            self.tx_count += 1
        else:
            self.rx_count += 1
        return pdu

    def _trace_connect(self, connected: bool) -> None:
        self.clients = max(0, self.clients + (1 if connected else -1))
        self.events.put(("clients", self.clients))

    # ---------------------------------------------------------------- local edits
    def set_values(self, kind: str, address: int, values) -> None:
        """Change values from the GUI (works with the server stopped or running)."""
        values = self._norm(kind, values)
        if address < 0 or address + len(values) > TABLE_SIZE:
            raise ValueError("address out of range")
        self.model[kind][address:address + len(values)] = values
        loop, server = self._loop, self._server
        if loop is None or server is None or loop.is_closed():
            return
        try:
            fut = asyncio.run_coroutine_threadsafe(
                server.async_setValues(0, TABLES[kind].local_fc, address, values), loop)
            fut.add_done_callback(self._log_push_error)
        except RuntimeError:
            pass

    @staticmethod
    def _log_push_error(fut) -> None:
        if not fut.cancelled() and fut.exception():
            print("Local write to running server failed:", fut.exception(), file=sys.stderr)

    def clear(self, kind: str) -> None:
        self.set_values(kind, 0, [0] * TABLE_SIZE)

    def export_sparse(self) -> dict:
        out: dict = {"format": "modbus-sim", "version": 1}
        for kind in TABLES:
            out[kind] = {str(i): int(v) for i, v in enumerate(self.model[kind]) if v}
        return out

    def import_sparse(self, data) -> None:
        if not isinstance(data, dict) or data.get("format") != "modbus-sim":
            raise ValueError("Not a Modbus simulator file.")
        for kind in TABLES:
            full = [0] * TABLE_SIZE
            for key, val in (data.get(kind) or {}).items():
                idx = int(key)
                if 0 <= idx < TABLE_SIZE:
                    full[idx] = int(val)
            self.set_values(kind, 0, full)


# ==========================================================================
# 2. SMALL GUI HELPERS
# ==========================================================================
class ToolTip:
    def __init__(self, widget: tk.Widget, text: str) -> None:
        self.widget, self.text, self.tip = widget, text, None
        widget.bind("<Enter>", self._show, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _show(self, _event=None) -> None:
        if self.tip:
            return
        self.tip = tk.Toplevel(self.widget)
        self.tip.wm_overrideredirect(True)
        self.tip.wm_geometry(f"+{self.widget.winfo_rootx() + 8}+{self.widget.winfo_rooty() + self.widget.winfo_height() + 4}")
        tk.Label(self.tip, text=self.text, background="#ffffe0", foreground="black",
                 relief="solid", borderwidth=1, padx=5, pady=2).pack()

    def _hide(self, _event=None) -> None:
        if self.tip:
            self.tip.destroy()
            self.tip = None


class CommsDialog(tk.Toplevel):
    """Modal, grid-based settings dialog: labels and fields share aligned columns."""

    def __init__(self, parent: tk.Misc, title: str) -> None:
        super().__init__(parent)
        self.title(title)
        self.transient(parent)
        self.resizable(False, False)
        self.result: dict | None = None
        self.body = ttk.Frame(self, padding=(16, 14, 16, 12))
        self.body.grid(sticky="nsew")
        self.body.columnconfigure(1, weight=1, minsize=210)
        self._row = 0
        self.protocol("WM_DELETE_WINDOW", self.cancel)
        self.bind("<Escape>", lambda _e: self.cancel())

    def add_row(self, label: str, widget: tk.Widget) -> tk.Widget:
        ttk.Label(self.body, text=label).grid(row=self._row, column=0, sticky="w", padx=(0, 14), pady=4)
        widget.grid(row=self._row, column=1, sticky="ew", pady=4)
        self._row += 1
        return widget

    def finish(self) -> None:
        buttons = ttk.Frame(self.body)
        buttons.grid(row=self._row, column=0, columnspan=2, sticky="e", pady=(14, 0))
        ttk.Button(buttons, text="OK", width=10, command=self.ok, default="active").grid(row=0, column=0, padx=(0, 6))
        ttk.Button(buttons, text="Cancel", width=10, command=self.cancel).grid(row=0, column=1)
        self.bind("<Return>", lambda _e: self.ok())
        self.update_idletasks()
        p = self.master
        x = p.winfo_rootx() + (p.winfo_width() - self.winfo_reqwidth()) // 2
        y = p.winfo_rooty() + (p.winfo_height() - self.winfo_reqheight()) // 3
        self.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        self.wait_visibility()
        self.grab_set()
        self.focus_set()

    def ok(self) -> None:          # overridden
        self.destroy()

    def cancel(self) -> None:
        self.result = None
        self.destroy()

    def show(self) -> dict | None:
        self.wait_window(self)
        return self.result


class TcpDialog(CommsDialog):
    def __init__(self, parent, current: dict) -> None:
        super().__init__(parent, "Ethernet TCP/IP Settings")
        addrs = ["0.0.0.0", "127.0.0.1"]
        try:
            for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
                if ip not in addrs:
                    addrs.append(ip)
        except OSError:
            pass
        self.host = self.add_row("Local IP", ttk.Combobox(self.body, values=addrs))
        self.host.set(current["host"])
        self.port = self.add_row("Port", ttk.Entry(self.body))
        self.port.insert(0, str(current["port"]))
        self.finish()
        self.host.focus_set()

    def ok(self) -> None:
        host = self.host.get().strip()
        try:
            port = int(self.port.get())
            if not (1 <= port <= 65535) or not host:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid settings", "Enter an IP address and a port between 1 and 65535.", parent=self)
            return
        self.result = {"host": host, "port": port}
        self.destroy()


class SerialDialog(CommsDialog):
    PARITY = {"None": "N", "Even": "E", "Odd": "O"}

    def __init__(self, parent, current: dict) -> None:
        super().__init__(parent, "RS-232 MODBUS PLC Sim-server Settings")
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except Exception:                                   # noqa: BLE001
            ports = []
        if not ports:
            ports = [f"COM{i}" for i in range(1, 9)] if sys.platform == "win32" else ["/dev/ttyS0", "/dev/ttyUSB0"]
        self.port = self.add_row("Port", ttk.Combobox(self.body, values=sorted(ports, key=natural_key)))
        self.port.set(current["port"])
        self.baud = self.add_row("Baud rate", ttk.Combobox(
            self.body, values=["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"]))
        self.baud.set(str(current["baudrate"]))
        self.bits = self.add_row("Data bits", ttk.Combobox(self.body, values=["7", "8"], state="readonly"))
        self.bits.set(str(current["bytesize"]))
        self.parity = self.add_row("Parity", ttk.Combobox(self.body, values=list(self.PARITY), state="readonly"))
        self.parity.set({v: k for k, v in self.PARITY.items()}[current["parity"]])
        self.stop = self.add_row("Stop bits", ttk.Combobox(self.body, values=["1", "2"], state="readonly"))
        self.stop.set(str(current["stopbits"]))
        self.finish()
        self.port.focus_set()

    def ok(self) -> None:
        port = self.port.get().replace("*", "").strip()
        try:
            baud = int(self.baud.get())
            if baud <= 0 or not port:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid settings", "Enter a serial port and a valid baud rate.", parent=self)
            return
        self.result = {"port": port, "baudrate": baud, "bytesize": int(self.bits.get()),
                       "parity": self.PARITY[self.parity.get()], "stopbits": int(self.stop.get())}
        self.destroy()


# ==========================================================================
# 3. GUI APPLICATION
# ==========================================================================
class ModbusSimulatorApp:
    LED_OFF, LED_ON = "#5a0000", "#ff2a2a"
    BIT_OFF_BG, BIT_OFF_FG = "#0b4d0b", "#d5ecd5"
    BIT_ON_BG, BIT_ON_FG = "#19d219", "#000000"

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.backend = ModbusSimulatorBackend()
        self.kind = "hr"
        self.running = False
        self.error_text = ""
        self.settings = {
            "tcp": {"host": "0.0.0.0", "port": 502},
            "serial": {"port": "COM1" if sys.platform == "win32" else "/dev/ttyS0",
                       "baudrate": 19200, "bytesize": 8, "parity": "N", "stopbits": 1},
        }
        self._edit: tuple | None = None
        self._last_rx = self._last_tx = 0
        self._last_status = ""

        root.columnconfigure(0, weight=1)
        root.rowconfigure(2, weight=1)
        self._build_style()
        self._build_status_bar()
        self._build_controls()
        self._build_table()
        self._build_bit_strip()

        self.refresh_table()
        self.refresh_strip()
        self._update_title()
        self._update_status()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.update_idletasks()
        root.minsize(root.winfo_reqwidth(), 420)
        root.after(POLL_MS, self._poll)

    # ------------------------------------------------------------------ layout
    def _build_style(self) -> None:
        style = ttk.Style(self.root)
        if sys.platform.startswith("linux") and "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Treeview", rowheight=22)
        self._bg = style.lookup("TFrame", "background") or self.root.cget("background")

    def _build_status_bar(self) -> None:
        bar = ttk.Frame(self.root, padding=(8, 8, 8, 2))
        bar.grid(row=0, column=0, sticky="ew")
        # reserve room for the widest status string (measured with the real font, so it is
        # correct at any Windows DPI scaling) - the label itself then never forces the width
        widest = tkfont.nametofont("TkDefaultFont").measure(
            "Connected (00) : (received/sent) (9999999/9999999) Serv. listening.") + 12
        bar.columnconfigure(0, weight=1, minsize=widest)

        self.status_var = tk.StringVar()
        self.status_lbl = ttk.Label(bar, textvariable=self.status_var, width=1)   # width=1: text clips instead of widening the window
        self.status_lbl.grid(row=0, column=0, sticky="ew")

        self.led_canvas = tk.Canvas(bar, width=44, height=26, bg=self._bg, highlightthickness=0, bd=0)
        self.led_canvas.grid(row=0, column=1, padx=(8, 4))
        self.led_rx = self.led_canvas.create_oval(4, 7, 18, 21, fill=self.LED_OFF, outline="#300000")
        self.led_tx = self.led_canvas.create_oval(26, 7, 40, 21, fill=self.LED_OFF, outline="#300000")
        ToolTip(self.led_canvas, "Left: request received   Right: response sent")

        tools = [("↓", 3, self.load_file, "Load register values from a file"),
                 ("↑", 3, self.save_file, "Save register values to a file"),
                 ("Eraser", 9, self.erase_table, "Clear the table currently shown"),
                 ("Set Comms", 9, self.open_comms_setup, "Configure communication and (re)start the server"),
                 ("Toggle", 9, self.toggle_comms, "Start / stop the server with the last settings")]
        for i, (text, width, cmd, tip) in enumerate(tools, start=2):
            btn = ttk.Button(bar, text=text, width=width, command=cmd)
            btn.grid(row=0, column=i, padx=(4, 0), sticky="ns")
            ToolTip(btn, tip)

    def _build_controls(self) -> None:
        ctl = ttk.Frame(self.root, padding=(8, 4))
        ctl.grid(row=1, column=0, sticky="ew")
        ctl.columnconfigure(7, weight=1)                      # spacer -> "Prot" hugs the right edge

        self.addr_var = tk.StringVar(value="Dec")
        ttk.Label(ctl, text="Address:").grid(row=0, column=0)
        ttk.Radiobutton(ctl, text="Hex", variable=self.addr_var, value="Hex", command=self.refresh_table).grid(row=0, column=1)
        ttk.Radiobutton(ctl, text="Dec", variable=self.addr_var, value="Dec", command=self.refresh_table).grid(row=0, column=2)

        ttk.Label(ctl, text="I/O:").grid(row=0, column=3)
        self.io_combo = ttk.Combobox(ctl, values=[t.label for t in TABLES.values()], width=24, state="readonly")
        self.io_combo.set(TABLES["hr"].label)
        self.io_combo.grid(row=0, column=4)
        self.io_combo.bind("<<ComboboxSelected>>", self._on_kind_changed)

        ttk.Label(ctl, text="Go to:").grid(row=0, column=5)
        self.goto_var = tk.StringVar()
        goto = ttk.Entry(ctl, textvariable=self.goto_var, width=9)
        goto.grid(row=0, column=6)
        goto.bind("<Return>", lambda _e: self.goto())
        ToolTip(goto, "Jump to an address (Dec: reference number as shown, e.g. 40100; Hex: protocol address)")

        ttk.Label(ctl, text="Prot:").grid(row=0, column=8)
        self.prot_combo = ttk.Combobox(ctl, values=["MODBUS RS-232", "MODBUS TCP/IP"], width=16, state="readonly")
        self.prot_combo.set("MODBUS TCP/IP")
        self.prot_combo.grid(row=0, column=9)
        self.prot_combo.bind("<<ComboboxSelected>>", lambda _e: self._update_title())

        for child in ctl.winfo_children():
            child.grid_configure(padx=(0, 6), pady=2)
        self.prot_combo.grid_configure(padx=0)                # last widget: flush with the table edge

    def _build_table(self) -> None:
        frame = ttk.Frame(self.root, padding=(8, 2))
        frame.grid(row=2, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        cols = ["addr"] + [f"c{i}" for i in range(COLS)]
        self.tree = ttk.Treeview(frame, columns=cols, show="headings", height=16, selectmode="browse")
        self.tree.heading("addr", text="Address", anchor="w")
        self.tree.column("addr", width=140, minwidth=110, anchor="w", stretch=False)
        for i in range(COLS):
            self.tree.heading(f"c{i}", text=f"+{i}", anchor="center")
            self.tree.column(f"c{i}", width=62, minwidth=48, anchor="center", stretch=True)

        self.vbar = ttk.Scrollbar(frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=self._on_yscroll)
        self.tree.grid(row=0, column=0, sticky="nsew")
        self.vbar.grid(row=0, column=1, sticky="ns")
        self.tree.bind("<Double-1>", self._begin_edit)

        for row in range(ROWS):
            self.tree.insert("", "end", iid=str(row), values=self._row_values(row))

    def _build_bit_strip(self) -> None:
        strip = ttk.LabelFrame(self.root, text=f"Coils 00001-{STRIP_BITS:05d}  (click to toggle)", padding=(6, 4))
        strip.grid(row=3, column=0, sticky="ew", padx=8, pady=(6, 8))
        per_row = STRIP_BITS // 2
        for c in range(per_row):
            strip.columnconfigure(c, weight=1, uniform="bit")
        self.bit_labels: list[tk.Label] = []
        for i in range(STRIP_BITS):
            lbl = tk.Label(strip, text=f"{i + 1:02d}", font=("TkDefaultFont", 8), width=2, bd=1,
                           relief="solid", cursor="hand2")
            lbl.grid(row=i // per_row, column=i % per_row, sticky="ew", padx=1, pady=1, ipady=2)
            lbl.bind("<Button-1>", lambda _e, n=i: self._toggle_coil(n))
            self.bit_labels.append(lbl)

    # ------------------------------------------------------------------ table
    def _fmt(self, value) -> str:
        if TABLES[self.kind].is_bit:
            return "1" if value else "0"
        return f"{value:04X}" if self.addr_var.get() == "Hex" else str(value)

    def _row_values(self, row: int) -> list[str]:
        first = row * COLS
        last = min(first + COLS, TABLE_SIZE) - 1
        if self.addr_var.get() == "Hex":
            addr = f"{first:04X}-{last:04X}"
        else:
            addr = f"{reference_number(self.kind, first)}-{reference_number(self.kind, last)}"
        cells = [self._fmt(v) for v in self.backend.model[self.kind][first:last + 1]]
        return [addr] + cells + [""] * (COLS - len(cells))

    def refresh_table(self) -> None:
        self._end_edit(commit=False)
        for row in range(ROWS):
            self.tree.item(str(row), values=self._row_values(row))

    def _refresh_rows(self, rows) -> None:
        for row in rows:
            self.tree.item(str(row), values=self._row_values(row))

    def refresh_strip(self) -> None:
        for i, lbl in enumerate(self.bit_labels):
            on = self.backend.model["co"][i]
            lbl.configure(bg=self.BIT_ON_BG if on else self.BIT_OFF_BG,
                          fg=self.BIT_ON_FG if on else self.BIT_OFF_FG)

    def _on_kind_changed(self, _event=None) -> None:
        self.kind = LABEL_TO_KIND[self.io_combo.get()]
        self.refresh_table()
        self.tree.yview_moveto(0)

    def _on_yscroll(self, first, last) -> None:
        self.vbar.set(first, last)
        self._end_edit(commit=False)

    def goto(self) -> None:
        text = self.goto_var.get().strip()
        try:
            if self.addr_var.get() == "Hex":
                address = int(text, 16)
            else:
                v = int(text)
                prefix = int(TABLES[self.kind].prefix)
                if prefix and v >= prefix * 100000:
                    v -= prefix * 100000
                elif prefix and v >= prefix * 10000:
                    v -= prefix * 10000
                address = v - 1
        except ValueError:
            self.root.bell()
            return
        address = min(max(address, 0), TABLE_SIZE - 1)
        row = str(address // COLS)
        self.tree.yview_moveto(int(row) / ROWS)
        self.tree.selection_set(row)
        self.tree.focus(row)

    # ------------------------------------------------------------------ editing
    def _begin_edit(self, event) -> None:
        self._end_edit(commit=False)
        if self.tree.identify_region(event.x, event.y) != "cell":
            return
        item = self.tree.identify_row(event.y)
        col = self.tree.identify_column(event.x)
        idx = int(col[1:]) - 2
        if not item or idx < 0:
            return
        address = int(item) * COLS + idx
        if address >= TABLE_SIZE:
            return
        kind = self.kind
        if TABLES[kind].is_bit:                               # bits: double-click toggles
            self._write(kind, address, [not self.backend.model[kind][address]])
            return
        bbox = self.tree.bbox(item, col)
        if not bbox:
            return
        x, y, w, h = bbox
        entry = ttk.Entry(self.tree, justify="center")
        entry.insert(0, self._fmt(self.backend.model[kind][address]))
        entry.select_range(0, "end")
        entry.place(x=x, y=y, width=w, height=h)
        entry.focus_set()
        entry.bind("<Return>", lambda _e: self._end_edit(True))
        entry.bind("<Escape>", lambda _e: self._end_edit(False))
        entry.bind("<FocusOut>", lambda _e: self._end_edit(True))
        self._edit = (entry, kind, address)

    def _end_edit(self, commit: bool) -> None:
        if not self._edit:
            return
        entry, kind, address = self._edit
        self._edit = None                                     # first: FocusOut re-entrancy guard
        text = entry.get().strip()
        entry.destroy()
        if not commit:
            return
        try:
            value = int(text, 16 if self.addr_var.get() == "Hex" else 10)
            if -32768 <= value < 0:
                value &= 0xFFFF                               # accept signed input
            if not 0 <= value <= 0xFFFF:
                raise ValueError
        except ValueError:
            self.root.bell()
            return
        self._write(kind, address, [value])

    def _write(self, kind: str, address: int, values: list) -> None:
        self.backend.set_values(kind, address, values)
        if kind == self.kind:
            self._refresh_rows({address // COLS})
        if kind == "co" and address < STRIP_BITS:
            self.refresh_strip()

    def _toggle_coil(self, index: int) -> None:
        self._write("co", index, [not self.backend.model["co"][index]])

    # ------------------------------------------------------------------ toolbar actions
    def erase_table(self) -> None:
        label = TABLES[self.kind].label
        if messagebox.askyesno("Eraser", f"Set every value of\n'{label}'\nto 0?", parent=self.root):
            self.backend.clear(self.kind)
            self.refresh_table()
            self.refresh_strip()

    def save_file(self) -> None:
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".json",
                                            filetypes=[("Simulator data", "*.json")])
        if path:
            try:
                with open(path, "w", encoding="utf-8") as fh:
                    json.dump(self.backend.export_sparse(), fh, indent=1)
            except OSError as exc:
                messagebox.showerror("Save failed", str(exc), parent=self.root)

    def load_file(self) -> None:
        path = filedialog.askopenfilename(parent=self.root, filetypes=[("Simulator data", "*.json")])
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as fh:
                self.backend.import_sparse(json.load(fh))
        except (OSError, ValueError) as exc:
            messagebox.showerror("Load failed", str(exc), parent=self.root)
            return
        self.refresh_table()
        self.refresh_strip()

    def _protocol_key(self) -> str:
        return "tcp" if "TCP/IP" in self.prot_combo.get() else "serial"

    def open_comms_setup(self) -> None:
        key = self._protocol_key()
        dlg = TcpDialog(self.root, self.settings["tcp"]) if key == "tcp" else SerialDialog(self.root, self.settings["serial"])
        result = dlg.show()
        if result:
            self.settings[key] = result
            self._start_server()

    def toggle_comms(self) -> None:
        if self.running or self.backend.is_alive():
            self.backend.stop()
            self.running = False
            self._update_status()
        else:
            self._start_server()

    def _start_server(self) -> None:
        self.error_text = ""
        key = self._protocol_key()
        if key == "tcp":
            self.backend.start_tcp(**self.settings["tcp"])
        else:
            self.backend.start_serial(**self.settings["serial"])
        self._update_title()
        self._update_status(starting=True)

    # ------------------------------------------------------------------ status / title
    def _update_title(self) -> None:
        if self._protocol_key() == "tcp":
            self.root.title(f"MODBUS Eth. TCP/IP PLC - Simulator (port: {self.settings['tcp']['port']})")
        else:
            s = self.settings["serial"]
            self.root.title(f"MODBUS RS-232 PLC - Simulator ({s['port']} @ {s['baudrate']})")

    def _update_status(self, starting: bool = False) -> None:
        b = self.backend
        if self.error_text:
            text, color = "Server error.  Fix the settings, then use Set Comms.", "#b00000"
        elif starting and not self.running:
            text, color = "Starting server ...", ""
        elif self.running:
            conn = f"Connected ({b.clients}) : " if self._protocol_key() == "tcp" else ""
            text = f"{conn}(received/sent) ({b.rx_count}/{b.tx_count}) Serv. listening."
            color = "#006400"
        else:
            text, color = "Server stopped.  Use Set Comms or Toggle.", ""
        if text != self._last_status:
            self._last_status = text
            self.status_var.set(text)
            self.status_lbl.configure(foreground=color)

    # ------------------------------------------------------------------ back-end polling
    def _poll(self) -> None:
        self.root.after(POLL_MS, self._poll)                  # re-arm first: never lose the timer
        dirty_rows: set[int] = set()
        full = strip = False
        error: str | None = None

        for _ in range(20000):                                # bounded drain per tick
            try:
                ev = self.backend.events.get_nowait()
            except queue.Empty:
                break
            tag = ev[0]
            if tag == "write":
                _, kind, addr, n = ev
                if kind == self.kind:
                    if n > 2000:
                        full = True
                    else:
                        dirty_rows.update(range(addr // COLS, (addr + n - 1) // COLS + 1))
                if kind == "co" and addr < STRIP_BITS:
                    strip = True
            elif tag == "started":
                self.running, self.error_text = True, ""
            elif tag == "stopped":
                self.running = False
            elif tag == "error":
                self.running, self.error_text, error = False, ev[1], ev[1]

        if full:
            self.refresh_table()
        elif dirty_rows:
            self._refresh_rows(dirty_rows)
        if strip or full:
            self.refresh_strip()

        rx, tx = self.backend.rx_count, self.backend.tx_count
        self.led_canvas.itemconfig(self.led_rx, fill=self.LED_ON if rx != self._last_rx else self.LED_OFF)
        self.led_canvas.itemconfig(self.led_tx, fill=self.LED_ON if tx != self._last_tx else self.LED_OFF)
        self._last_rx, self._last_tx = rx, tx

        self._update_status()
        if error:
            messagebox.showerror("Modbus server", error, parent=self.root)

    def on_close(self) -> None:
        self.backend.stop()
        self.root.destroy()


# ==========================================================================
# 4. ENTRY POINT
# ==========================================================================
def _enable_dpi_awareness() -> None:
    """Crisp text and correct widget sizes on scaled Windows displays."""
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:                                      # noqa: BLE001
            pass


def main() -> None:
    _enable_dpi_awareness()
    root = tk.Tk()
    ModbusSimulatorApp(root)
    root.mainloop()


if __name__ == "__main__":
    try:
        main()
    except Exception:                                          # noqa: BLE001
        traceback.print_exc()
        input("Press Enter to close...")                       # keep console open on a crash
