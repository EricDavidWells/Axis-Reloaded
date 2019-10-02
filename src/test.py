###############################################################################
#	Author:			Sebastien Parent-Charette (support@robotshop.com)
#	Version:		1.0.0
#	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#	
#	Desscription:	An example using the LSS and the Python module.
###############################################################################

# Import required liraries
import time
import serial

# Import LSS library
import lss
import lss_const as lssc

# Constants
#CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
CST_LSS_Port = "COM3"				# For windows platforms
CST_LSS_Baud = lssc.LSS_DefaultBaud

# Create and open a serial port
lss.initBus(CST_LSS_Port, CST_LSS_Baud)

# Create LSS objects
myLSS1 = lss.LSS(0)
myLSS2 = lss.LSS(1)

#myLSS1.move(-300)
#myLSS2.move(300)

print("myLSS1; position = " + str(myLSS1.getPosition()))
print("myLSS2; position = " + str(myLSS2.getPosition()))
print("myLSS1; model = " + str(myLSS1.getModel()))
myLSS1.setColorLED(lssc.LSS_LED_Green, lssc.LSS_SetConfig)
#myLSS1.reset()
#myLSS2.reset()

# Destroy objects
del myLSS1
del myLSS2

# Destroy the bus
lss.closeBus()

### EOF #######################################################################
