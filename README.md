# EnergyMeter
Energy meter with PIC 16F18875 microcontroller, XBEE PRO S1 (API1 mode), SDM120 and BMP180.

This project contains the first prototype of an energy meter using SDM120M module. It's a PROTOTYPE only for testing purposes, a lot of functionalities are missing.
- ELECTRICAL PROTECTION missing.
- No check on components failure.

Description.
SDM120 has an output generationg a pulse each 1 W/h. It's connected to an input, RC5, counting all pulses.
From manual pulse length is about 60ms so no need for interrupts.

From PIC side. Controller is connected to a BMP180 via I2C, temperature and pressure are read and sent back to a main system via a XBEE PRO S1 module.
XBEE (node 5) is connected via USART (9600 bps). AP1 mode is used to detect success of sending operations.
Data are sent using a sort of index/table format. One byte defines the type of table sent. At the moment two tables are generated:
- coefficients of BMP180. Read at power up and immediately sent (index 0x0B)
- temperature and pressure (index 0x0A). Values are sent periodically.
TMR0 interrupt (one interrupt each 10ms) is used for all timing functions.

On main side a Raspberry Pi connected to another XBEE PRO (node 1, 19200 bps). A Python program detects and mask data, saving Log, and diagnostics in text files.
Data are saved in a CSV file ready (I hope...) to be used by R. One record is saved every hour. Format:
- Date and time
- Pulses from controller (1 pulse is equal to 1 W/h)
- Temperature [°C * 10]
- Pressure [mbar]

Programs of both XBEE are in the project.
Schematics is present.

Bye
