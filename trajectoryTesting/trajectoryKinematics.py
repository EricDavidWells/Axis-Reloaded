import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


from timeit import default_timer as timer
from scipy.optimize import fsolve

class trajectoryKinematics():
    def __init__(self, claw, radius, points, depth):
        self.claw = claw
        self.radius = radius
        self.points = points
        self.depth = depth

        self.jointDesired1 = 0
        self.jointDesired2 = 0
        self.jointDesired3 = 0

        self.jointActual1 = 0
        self.jointActual2 = 0
        self.jointActual3 = 0

        self.x = 90
        self.y = 100
        self.z = 150

    def forwardKinematics(self, J):
        j1, j2, j3 = J

        f1 = -self.x + 5*math.sin(j1)*(20*math.cos(j2 + j3) - \
                       3*math.sin(j2 + j3) + 12*math.cos(j2))
        f2 = -self.y + 156 - 5*pow(409,0.5)*math.cos(j2 + j3 - \
                       math.atan(20/3)) - 60*math.sin(j2)
        f3 = -self.z + 60*math.cos(j1)*math.cos(j2) + \
                       100*math.cos(j1)*math.cos(j2)*math.cos(j3) - \
                       15*math.cos(j1)*math.cos(j2)*math.sin(j3) -  \
                       15*math.cos(j1)*math.cos(j3)*math.sin(j2) -  \
                       100*math.cos(j1)*math.sin(j2)*math.sin(j3) + 41

        return (f1, f2, f3)

    def inverseKinematics(self):
        guess = (self.jointActual1, self.jointActual2, self.jointActual3)
        j1, j2, j3 = fsolve(self.forwardKinematics, guess)
        self.jointDesired1 = j1
        self.jointDesired2 = j2
        self.jointDesired3 = j3

    def trajectory(self, plotFlag):
        th = np.linspace((2*np.pi/5)*self.claw, (2*np.pi/5)*(self.claw+1), self.points)
        idx = np.linspace(-self.points/2, self.points/2, self.points)

        x = self.radius * np.cos(th)
        y = self.radius * np.sin(th)
        z = (self.depth)/(self.points/2)**2 * idx**2 - self.depth

        if plotFlag is True:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(x,y,np.linspace(0, 0, self.points))
            plt.plot(x,y,z)
            plt.axis('equal')
            plt.show()

def main():
    claw0 = trajectoryKinematics(0, 230, 200, 30)

    claw0.trajectory(False)

    start = timer()
    claw0.inverseKinematics()
    end = timer()

    print(claw0.jointDesired1, claw0.jointDesired2, claw0.jointDesired3)
    print(end - start)

if __name__ == '__main__':
    main()
