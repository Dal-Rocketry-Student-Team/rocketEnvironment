# list_ports.py
from serial.tools import list_ports
for ports in list_ports.comports():
    print(ports.device, "-", ports.description)
