#
#	Author:			Sebastien Parent-Charette (support@robotshop.com)
#	Version:		1.0.0
#	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#	
#	Desscription:	Moves one LSS using the position of a second LSS.
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

# Create two LSS object; one for output (ID=0), one for input (ID=1)
myLSS_Output = lss.LSS(0)
myLSS_Input = lss.LSS(1)

# Initialize LSS output to position 0.0
myLSS_Output.move(0)

# Wait for it to get there
time.sleep(2)

# Lower output servo stiffness
myLSS_Output.setAngularStiffness(4)
myLSS_Output.setMaxSpeedRPM(15)

# Make input servo limp (no active resistance)
myLSS_Input.limp();

# Reproduces position of myLSS_Input on myLSS_Output
pos = 0
while 1:
	# Wait ~20 ms before sending another command (update at most 50 times per second)
	time.sleep(0.02)
	
	# Get position & check if it is valid (ex: servo missing)
	lastPos = pos
	posRead = myLSS_Input.getPosition()
	if(posRead is not None):
		pos = int(posRead,10)
		
		# Check if pos changed enough to warrant sending/showing (hysterisis of ±2)
		if ((pos < (lastPos - 2)) or (pos > (lastPos + 2))):
			# Send position to output servo and terminal
			print("Input @ " + str(pos))
			myLSS_Output.move(pos)

### EOF #######################################################################
