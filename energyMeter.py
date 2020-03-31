#!/usr/bin/env python3
import serial
import struct
import time
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee16BitAddress, XBeeException
from datetime import datetime
import sys
import os

OSS=2 #IMPORTANT! Oversampling setting (fixed in CPU program)

#Coefficients values. These values will be overwritten at remote node startup.
#Values are assigned here for debugging.
AC1=8438
AC2=1181
AC3=-14570
AC4=34094
AC5=25298
AC6=19799
B1=6515
B2=47
MB=-32768
MC=-11786
MD=2638
param={"AC1":AC1, "AC2":AC2, "AC3":AC3, "AC4":AC4, "AC5":AC5, "AC6":AC6, "B1":B1, "B2":B2, "MB":MB, "MC":MC, "MD":MD}

#***From 2 bytes to INT***
def bytes_to_short(B1, B2):
	"""Converts two bytes to short. First byte is MSB."""
	a=[B1, B2]
	values = bytearray(a)
	b=struct.unpack('>h', values)
	return b[0]
#*************************

def calculateTempPress(avgTemperature, avgPressure, parameters):
	X1 = ((avgTemperature - param["AC6"]) * param["AC5"]) / 32768
	X2 = (param["MC"] * 2048) / (X1 + param["MD"])
	B5 = X1 + X2
	T = (B5 + 8) / 16	
	
	B6 = B5 - 4000
	X1 = (param["B2"]*B6*(B6/4096)) / 2048
	X2 = (param["AC2"]*B6) / 2048
	X3 = X1 + X2
	B3 = (((param["AC1"]*4 + X3) * (2**OSS))+2) / 4
	X1 = (param["AC3"] * B6) / 8192
	X2 = (param["B1"]*(B6*(B6 /4096))) / 65536
	X3=((X1+X2)+2) / 4
	B4 = (param["AC4"] * (X3 + 32768)) /32768
	B7 = int((avgPressure-B3)*(50000>>OSS))
	if (B7 < 0x80000000):
		P=(B7*2)/B4
	else:
		P=(B7/B4)*2
	X1=(P/256)*(P/256)
	X1=(X1*3038)/65536
	X2=(-7357*P)/65536
	P=P+(X1+X2+3791)/16 #Pa
	P=P/100 #Pa --> mbar
	return T,P
#*************************

#*****Define callback*****
def my_data_received_callback(xbee_message):
    #Uso le variabili globali: non riesco a ritornare valori da
    #funzione di callback
	global g_ReceivedPower, g_dataReady, g_coeffReady, g_address, g_ReceivedTemperature, g_receivedPressure
	global g_xbee_recv_data
	
	g_address = xbee_message.remote_device.get_16bit_addr()
	
	g_xbee_recv_data = xbee_message.data
	
	if (xbee_message.data[0] == 0x0A): #Questo è il codice per i dati
		g_ReceivedPower = bytes_to_short(xbee_message.data[3], xbee_message.data[2])
		g_ReceivedTemperature = bytes_to_short(xbee_message.data[5], xbee_message.data[4]) #(xbee_message.data[4] | (xbee_message.data[5] << 8) )
		g_receivedPressure = int(((xbee_message.data[7] << 16) + (xbee_message.data[6] << 8))) >> (8-OSS)
		g_dataReady = 1
	elif (xbee_message.data[0] == 0x0B): #Coefficienti
		g_ReceivedPower = 0
		g_coeffReady = 1
		
	else:
		print("Unknown message type")
		g_ReceivedPower = 0
		
#*******************************

actualDT = datetime.now()
previousDT = actualDT

os_Name = os.name
if (os_Name == "posix"):
	PORT = "/dev/ttyUSB0"
	DIAG_FILE = "/home/pi/EnergyMeter/Diag.txt"
	LOG_FILE = "/home/pi/EnergyMeter/Log.txt"
elif (os_Name == "nt"):
	PORT = "COM10"
	DIAG_FILE = "C:\\EnergyMeter\\Diag.txt"
	LOG_FILE = "C:\\EnergyMeter\\Log.txt"
else:
    print("Unsupported operating system: ", os_Name)
    quit()
	
print("Detected OS: ", os_Name)

fd_diag = open(DIAG_FILE,"a")
stringa = str(actualDT.strftime("%Y-%m-%d-%H:%M:%S;")) + "Program started\n"
fd_diag.write(stringa)
fd_diag.close()

g_dataReady = 0
g_coeffReady = 0
g_address = 0

print("Detect device on port: ", PORT)

device = XBeeDevice(PORT, 19200) 

device.open()

remote_device = RemoteXBeeDevice(device, XBee16BitAddress.from_hex_string("01"))

a=device.get_firmware_version()  

# Add the callback.
device.add_data_received_callback(my_data_received_callback)

node_id = device.get_node_id()
print("Node ID: ", node_id, "FW version: ", a)

powerCounter = 0
pr_powerCounter = 0;
pr_minuti = 0
hourPowerCounter = 0
hourTemperatureCounter = 0
hourPressureCounter = 0
loopCtr = 0

while True:
	actualDT = datetime.now()

	if (g_dataReady):
		g_dataReady = 0
		
		xbeeDT = datetime.now()
		data = str(xbeeDT.strftime("%Y-%m-%d"))
		ora = str(xbeeDT.strftime("%H:%M:%S"))

		powerCounter = powerCounter + g_ReceivedPower
		hourPowerCounter = hourPowerCounter + g_ReceivedPower
		#print(data, ora, "; Nodo ", g_address, ";%d [mw/h] " %g_ReceivedPower, ";Hour power ", hourPowerCounter, ";Power counter %d [imp]" %powerCounter)
		hourTemperatureCounter = hourTemperatureCounter + g_ReceivedTemperature
		hourPressureCounter = hourPressureCounter + g_receivedPressure
		prXbeeDT = xbeeDT #Serve?
		loopCtr = loopCtr + 1
		
	if (g_coeffReady):
		g_coeffReady = 0
		#***Write diagnostic file***
		fd_diag = open(DIAG_FILE,"a")
		stringa = str(actualDT.strftime("%Y-%m-%d-%H:%M:%S;")) + "Coefficients received\n"
		fd_diag.write(stringa)
		
		#Coefficients calculation
		#g_xbee_recv_data[0] is the type of message
		AC1 = bytes_to_int(g_xbee_recv_data[1], g_xbee_recv_data[2])
		AC2 = bytes_to_int(g_xbee_recv_data[3], g_xbee_recv_data[4])
		AC3 = bytes_to_int(g_xbee_recv_data[5], g_xbee_recv_data[6])
		AC4 = (g_xbee_recv_data[7] << 8) | g_xbee_recv_data[8] #Unsigned short
		AC5 = (g_xbee_recv_data[9] << 8) | g_xbee_recv_data[10] #Unsigned short
		AC6 = (g_xbee_recv_data[11] << 8) | g_xbee_recv_data[12] #Unsigned short
		B1 = bytes_to_int(g_xbee_recv_data[13], g_xbee_recv_data[14])
		B2 = bytes_to_int(g_xbee_recv_data[15], g_xbee_recv_data[16])
		MB = bytes_to_int(g_xbee_recv_data[17], g_xbee_recv_data[18])
		MC = bytes_to_int(g_xbee_recv_data[19], g_xbee_recv_data[20])
		MD = bytes_to_int(g_xbee_recv_data[21], g_xbee_recv_data[22])

		param={"AC1":AC1, "AC2":AC2, "AC3":AC3, "AC4":AC4, "AC5":AC5, "AC6":AC6, "B1":B1, "B2":B2, "MB":MB, "MC":MC, "MD":MD}
		
		stringa = "AC1 " + str(AC1) + ",AC2 " + str(AC2) + ",AC3 " + str(AC3)+"\n"
		fd_diag.write(stringa)
		stringa = "AC4 " + str(AC4) + ",AC5 " + str(AC5) + ",AC6 " + str(AC6)+"\n"
		fd_diag.write(stringa)
		stringa = "B1 " + str(B1) + ",B2 " + str(B2)+"\n"
		fd_diag.write(stringa)
		stringa = "MB " + str(MB) + ",MC " + str(MC) + ",MD " + str(MD)+"\n"
		fd_diag.write(stringa)
		fd_diag.close()
		
	minuti = actualDT.minute
	if ( (minuti == 0) and (pr_minuti == 0) ): #Data saved every hour in txt file. NO DB.
		
		fd_Log = open(LOG_FILE,"a")
		
		#Date;Time;Pulses during last hour
		if (loopCtr != 0):
			avgUT = hourTemperatureCounter / loopCtr
			avgUP = hourPressureCounter / loopCtr
				
		(T, P) = calculateTempPress(avgUT, avgUP, param) #Calculated temperature [0.1 °C]
		
		stringa = str(actualDT.strftime("%Y-%m-%d %H:%M:%S,")) + str(hourPowerCounter) + ",{0:.2f}".format(T) + ",{0:.2f}".format(P) + "\n"
		fd_Log.write(stringa)
		fd_Log.close()
		
		hourPowerCounter = 0
		hourTemperatureCounter = 0
		hourPressureCounter = 0
		loopCtr = 0
		
	pr_minuti = (minuti == 0)
	
	deltatime = actualDT - previousDT #timedelta
	
	previousDT = actualDT
	
device.close()