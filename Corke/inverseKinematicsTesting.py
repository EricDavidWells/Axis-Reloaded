import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from timeit import default_timer as timer
from scipy.optimize import fsolve
from scipy.optimize import root
from scipy.optimize import minimize


class Finger():
    def __init__(self, J_):

        self.J = np.array(J_)
        self.posActual = self.fkine(self.J)

    def fkine(self, J):
        j1, j2, j3 = J

        f1 = 5*math.sin(j1)*(20*math.cos(j2 + j3) - \
                       3*math.sin(j2 + j3) + 12*math.cos(j2))
        f2 = 156 - 5*pow(409,0.5)*math.cos(j2 + j3 - \
                       math.atan(20/3)) - 60*math.sin(j2)
        f3 = 60*math.cos(j1)*math.cos(j2) + \
                       100*math.cos(j1)*math.cos(j2)*math.cos(j3) - \
                       15*math.cos(j1)*math.cos(j2)*math.sin(j3) -  \
                       15*math.cos(j1)*math.cos(j3)*math.sin(j2) -  \
                       100*math.cos(j1)*math.sin(j2)*math.sin(j3) + 41

        return np.array([f1, f2, f3])

    def ikine(self, posDesired):

        guess = self.J
        iJ = minimize(self.ikine_err, guess, args=posDesired, tol=0.001)
        return iJ

    def ikine2(self, posDesired):

        guess = self.J
        iJ = fsolve(self.ikine_err2, guess, args=posDesired)
        # err = np.linalg.norm(self.fkine(iJ) - posDesired)
        return iJ

    def ikine3(self, posDesired):

        guess = self.J
        iJ = root(self.ikine_err2, guess, args=posDesired, tol=0.001)
        # err = np.linalg.norm(self.fkine(iJ) - posDesired)
        return iJ

    def ikine_err(self, J, posDesired):
        e = np.linalg.norm(self.fkine(J) - np.array(posDesired))
        return e

    def ikine_err2(self, J, posDesired):
        e = self.fkine(J) - np.array(posDesired)
        return e[0], e[1], e[2]



def main():
    finger = Finger([0, 0, 0])
    pos = [10, 100, 150]
    start = timer()
    iJ = finger.ikine(pos)
    end = timer()
    print(iJ.x, iJ.fun)
    print(end-start)

    start = timer()
    iJ = finger.ikine2(pos)
    end = timer()
    print(iJ)
    print(end-start)

    start = timer()
    iJ = finger.ikine3(pos)
    end = timer()
    print(iJ.x, np.linalg.norm(iJ.fun))
    print(end-start)

if __name__ == '__main__':
    main()
