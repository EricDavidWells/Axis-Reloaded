#
#	Author:			Sebastien Parent-Charette (support@robotshop.com)
#	Version:		1.0.0
#	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#	
#	Desscription:	Example of all the possible configurations for a LSS.
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

# Uncomment any configurations that you wish to activate
# You can see above each configuration a link to its description in the Lynxmotion wiki
# Note: If you change a configuration to the same value that is already set,
#       the LSS will ignore the operation since the value is not changed.

# *** Basic configurations ***
# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#H6.OriginOffsetAction28O29
###myLSS.setOriginOffset(0)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#H7.AngularRange28AR29
###myLSS.setAngularRange(1800, lssc.LSS_SetConfig)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#H12.MaxSpeedinDegrees28SD29
# Set maximum speed in (1/10 deg)/s
###myLSS.setMaxSpeed(600, lssc.LSS_SetConfig)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#H13.MaxSpeedinRPM28SR29
###myLSS.setMaxSpeedRPM(100, lssc.LSS_SetConfig)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#H14.LEDColor28LED29
# Options are:
# LSS_LED_Black = 0
# LSS_LED_Red = 1
# LSS_LED_Green = 2
# LSS_LED_Blue = 3
# LSS_LED_Yellow = 4
# LSS_LED_Cyan = 5
# LSS_LED_Magenta = 6
# LSS_LED_White = 7
###myLSS.setColorLED(lssc.LSS_LED_Black, lssc.LSS_SetConfig)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#H15.GyreRotationDirection28G29
# Options are:
# LSS_GyreClockwise = 1
# LSS_GyreCounterClockwise = -1
###myLSS.setGyre(lssc.LSS_GyreClockwise, lssc.LSS_SetConfig)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#H19.FirstA0Position28Degrees29
###myLSS.setFirstPosition(0)
###myLSS.clearFirstPosition()

# *** Advaned configurations ***
# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#HA1.AngularStiffness28AS29
###myLSS.setAngularStiffness(0, lssc.LSS_SetConfig)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#HA2.AngularHoldingStiffness28AH29
###myLSS.setAngularHoldingStiffness(4, lssc.LSS_SetConfig)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#HA3:AngularAcceleration28AA29
###myLSS.setAngularAcceleration(100, lssc.LSS_SetConfig)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#HA4:AngularDeceleration28AD29
###myLSS.setAngularDeceleration(100, lssc.LSS_SetConfig)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#HA5:MotionControl28EM29
###myLSS.setMotionControlEnabled(True)

# https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/#HA6.ConfigureLEDBlinking28CLB29
# Options are an arithmetic addition of the following values:
# Limp	1
# Holding	2
# Accelerating	4
# Decelerating	8
# Free	16
# Travelling	32
# Therefore, 0 = no blinking and 63 = always blinking
###myLSS.setBlinkingLED(0)

# Reset motor to complete change of configurations
myLSS.reset()

# Wait for reboot
time.sleep(2)

while 1:
	# Loop through each of the 8 LED color (black = 0, red = 1, ..., white = 7)
	for i in range (0, 7):
		# Set the color (session) of the LSS
		myLSS.setColorLED(i)
		# Wait 1 second per color change
		time.sleep(1)

### EOF #######################################################################
