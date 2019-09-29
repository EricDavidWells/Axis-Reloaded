import serial
import lss
import lss_const as lssc
import time

comport = 'COM6'
servoID1 = 21
servoID2 = 22
servoID3 = 23

baudrate = lssc.LSS_DefaultBaud
lss.initBus(comport, baudrate)

myLSS1 = lss.LSS(servoID1)
myLSS2 = lss.LSS(servoID2)
myLSS3 = lss.LSS(servoID3)

# myLSS1.setColorLED(1, lssc.LSS_SetConfig)
# myLSS2.setColorLED(2, lssc.LSS_SetConfig)
# myLSS3.setColorLED(3, lssc.LSS_SetConfig)

# myLSS.setOriginOffset(-900, lssc.LSS_SetConfig)
myLSS1.reset()
myLSS2.reset()
myLSS3.reset()
# myLSS.setOriginOffset(-900)
time.sleep(5)
myLSS1.move(0)
myLSS2.move(0)
myLSS3.move(0)

while True:
    # start = time.time()
    pos1 = myLSS1.getPosition()
    pos2 = myLSS2.getPosition()
    pos3 = myLSS3.getPosition()

    # end = time.time()
    print(pos1, pos2, pos3)
    time.sleep(0.1)