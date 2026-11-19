# Modbus Simulator

A Modbus slave (PLC) simulator with a Tkinter GUI. It serves **Modbus TCP/IP** and **Modbus RTU over a serial port**, and gives you a live view of all four data tables.

## Features

- Modbus TCP/IP and RTU serial slave, answering any unit ID
- All four tables with the full address space (0-65535): Holding Registers (4xxxx), Coil Outputs (0xxxx), Digital Inputs (1xxxx), Input Registers (3xxxx)
- Live grid: values written by a master appear immediately
- Double-click any cell to edit it; the change is visible to the master straight away
- Dec / Hex display, "Go to" address jump, and a clickable coil quick-view strip (coils 1-52)
- Save / load register values as JSON, and an eraser for the table on screen
- Supported function codes: 1, 2, 3, 4, 5, 6, 15, 16, 22, 23

## Requirements

- Python 3.10 or newer (with Tkinter)
- `pymodbus` 3.13 or newer
- `pyserial`

```
pip install -U pymodbus pyserial
```

## Run

```
python Modbus_SIM.py
```

1. Choose the protocol (`MODBUS TCP/IP` or `MODBUS RS-232`) in the **Prot** box.
2. Click **Set Comms**, enter the settings and press OK. The server starts.
3. **Toggle** starts or stops the server with the last settings.

Port 502 needs administrator rights on Linux and macOS; use a port above 1024 otherwise.

## Notes

- Tested against pymodbus 3.13.1, 3.14.0 and 3.15.0. Older versions are rejected at start-up with a clear message.
- Not tested on Windows or with real serial hardware.
