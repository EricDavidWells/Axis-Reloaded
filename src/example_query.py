#
#	Author:			Sebastien Parent-Charette (support@robotshop.com)
#	Version:		1.0.0
#	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#	
#	Desscription:	Basic example of reading values from the LSS and placing
#					them on the terminal.
#

# Import required liraries
import time
import serial

# Import LSS library
import lss
import lss_const as lssc

# Constants
#CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
CST_LSS_Port = "COM230"				# For windows platforms
CST_LSS_Baud = lssc.LSS_DefaultBaud

# Create and open a serial port
lss.initBus(CST_LSS_Port, CST_LSS_Baud)

# Create an LSS object
myLSS = lss.LSS(0)

while 1:
	# Get the values from LSS
	print("\r\nQuerying LSS...")
	pos = myLSS.getPosition()
	rpm= myLSS.getSpeedRPM()
	curr = myLSS.getCurrent()
	volt = myLSS.getVoltage()
	temp = myLSS.getTemperature()
	
	# Display the values in terminal
	print("\r\n---- Telemetry ----")
	print("Position  (1/10 deg) = " + str(pos));
	print("Speed          (rpm) = " + str(rpm));
	print("Curent          (mA) = " + str(curr));
	print("Voltage         (mV) = " + str(volt));
	print("Temperature (1/10 C) = " + str(temp));
	
	# Wait 1 second
	time.sleep(1)

### EOF #######################################################################
