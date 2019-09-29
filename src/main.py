import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from timeit import default_timer as timer
from scipy.optimize import fsolve
from scipy.optimize import root
from scipy.optimize import minimize
from math import cos, sin, pi
import serial
import lss
import lss_const as lssc
import time


class Finger():
    def __init__(self, servoarray):
        """

        :param J0: initial theta vector of finger
        """
        self.servos = servoarray
        self.J = np.array([0, 0, 0])

        # All kinematic constants defined
        self.Tsb0 = np.array([[1,0,0,0],
                             [0,1,0,41],
                             [0,0,1,201],
                             [0,0,0,1]])
        self.w1 = np.array([0, -1, 0]).reshape(3,1)
        self.p1 = np.array([0, 0, 41]).reshape(3,1)
        self.w2 = np.array([-1, 0, 0]).reshape(3,1)
        self.p2 = np.array([0, 56, 41]).reshape(3,1)
        self.w3 = np.array([1, 0, 0]).reshape(3,1)
        self.p3 = np.array([0, 56, 101]).reshape(3,1)
        self.Jlim = np.array([[-90, 90], [-30, 110], [-80, 120]])

    def fkine(self, theta):
        """
        Compute forward kinematics of finger end effector relative to finger base
        """
        Tsb = self.Tsbgen(theta)
        pos = Tsb[0:3, 3]
        return pos

    def ikine(self, posDesired):
        """
        Compute inverse kinematics joint angles of desired position of end effector relative to finger base
        """
        guess = self.J
        iJ = root(self.ikine_err, guess, args=posDesired, tol=0.001)
        return iJ.x, iJ.fun

    def ikine_err(self, J, posDesired):
        e = self.fkine(J) - np.array(posDesired)
        return e[0], e[1], e[2]

    def Tsbgen(self, theta):
        eX1 = self.eXgen(self.p1, self.w1, theta[0])
        eX2 = self.eXgen(self.p2, self.w2, theta[1])
        eX3 = self.eXgen(self.p3, self.w3, theta[2])
        Tsb = eX1@eX2@eX3@self.Tsb0
        return Tsb

    def eXgen(self, p, w, theta):
        z = np.cross(-w.reshape(1,3), p.reshape(1,3)).reshape(3,1)
        sw = skew(w)
        eS = np.eye(3) + skew(w)*sin(theta) + skew(w)@skew(w)*(1-cos(theta))
        G = np.eye(3) * theta + skew(w) * (1 - cos(theta)) + skew(w)@skew(w) * (theta - sin(theta))
        temp = np.hstack((eS, G@z))
        eX = np.vstack((temp, [0, 0, 0, 1]))
        return eX

    def move(self, J):
        Jdeg = J*180/pi
        for i in range(0, len(self.servos)):
            Jdeg[i] = np.clip(Jdeg[i], self.Jlim[i, 0], self.Jlim[i, 1])
            value = int(Jdeg[i]*10)
            self.servos[i].move(value)

    def reset(self):
        for i in range(0, len(self.servos)):
            self.servos[i].reset()
        time.sleep(3)

def skew(v):
        a, b, c = v[0,0], v[1,0], v[2,0]
        s = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
        return s


def main():
    comport = 'COM6'
    servoID1 = 21
    servoID2 = 22
    servoID3 = 23
    baudrate = lssc.LSS_DefaultBaud
    lss.initBus(comport, baudrate)
    myLSS1 = lss.LSS(servoID1)
    myLSS2 = lss.LSS(servoID2)
    myLSS3 = lss.LSS(servoID3)


    finger = Finger([myLSS1, myLSS2, myLSS3])
    # finger.reset()
    finger.J = np.array([0, 0, 0])
    finger.move(finger.J)

    # finger.servos[0].move(0)
    pos = [100, -25, 0]
    iJ, err = finger.ikine(pos)
    finger.move(iJ)
    print(iJ, err)

if __name__ == '__main__':
    main()
