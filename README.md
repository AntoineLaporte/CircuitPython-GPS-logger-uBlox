# CircuitPython-GPS-logger-uBlox
CircuitPython GPS logger uBlox with NAV-PVT message parsing

This is a Circuitpython program that has to be run with a uBlox compatible GPS chip running on a battery.
It is recording the data at 5Hz in a csv format made for speed sports.

boot.py is unmounting the CIRCUITPY drive USB is connected so we can write on the drive and mounting the drive when USB is not connected so the program can log the data on the drive.
This USB detection is made by a voltage divider made of two 100Kohm resistors

In the files there is also the Debugging .txt with the 2 procedures:
- one to unlock the drive by renaming the boot.py
- the other to format the drive and start with a clean install
