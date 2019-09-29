import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from timeit import default_timer as timer
from scipy.optimize import fsolve
from scipy.optimize import root
from scipy.optimize import minimize
from math import cos, sin


class Finger():
    def __init__(self, J0):
        """

        :param J0: initial theta vector of finger
        """

        self.J = np.array(J0)

        # All kinematic constants defined
        self.Tsb0 = np.array([[1,0,0,0],
                             [0,1,0,41],
                             [0,0,1,201],
                             [0,0,0,1]])
        self.w1 = np.array([0, 1, 0]).reshape(3,1)
        self.p1 = np.array([0, 0, 41]).reshape(3,1)
        self.w2 = np.array([1, 0, 0]).reshape(3,1)
        self.p2 = np.array([0, 56, 41]).reshape(3,1)
        self.w3 = np.array([1, 0, 0]).reshape(3,1)
        self.p3 = np.array([0, 56, 101]).reshape(3,1)

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
        eS = np.eye(3) + skew(w)*sin(theta) + skew(w)@skew(w)*(1-cos(theta))
        G = np.eye(3) * theta + skew(w) * (1 - cos(theta)) + skew(w)@skew(w) * (theta - sin(theta))
        temp = np.hstack((eS, G@z))
        eX = np.vstack((temp, [0, 0, 0, 1]))
        return eX


def skew(v):
        a, b, c = v
        s = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
        return s


def main():
    finger = Finger([0, 0, 0])
    pos = finger.fkine([1, 0, 0])
    iJ, err = finger.ikine(pos)
    print(iJ, err)

if __name__ == '__main__':
    main()
