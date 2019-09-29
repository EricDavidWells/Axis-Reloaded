import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from timeit import default_timer as timer
from scipy.optimize import fsolve

class trajectoryKinematics():
    def __init__(self, claw, radius, points, elevation, depth, cmdRate, travelTime, returnTime):
        self.claw = claw                # Claw number 0, 1, 2, ...
        self.radius = radius            # Radius of disk being spun
        self.points = points            # Number of data points for trajectory curve
        self.elevation = elevation      # Height of spinning disk
        self.depth = depth              # Depth of retraction of claw
        self.cmdRate = cmdRate          # How often to update in seconds
        self.travelTime = travelTime    # Amount of time claw is in contact with disk
        self.returnTime = returnTime    # Time in seconds claw is not in contact with disk

        self.jointDesired1 = 0
        self.jointDesired2 = 0
        self.jointDesired3 = 0

        self.jointActual1 = 0.5
        self.jointActual2 = 0.5
        self.jointActual3 = 0.5

        self.x = 0
        self.y = 0
        self.z = 0

    def forwardKinematics(self, J):
        j1, j2, j3 = J

        f1 = -self.xx + 5*math.sin(j1)*(20*math.cos(j2 + j3) - \
                        3*math.sin(j2 + j3) + 12*math.cos(j2))
        f2 = -self.yy + 156 - 5*pow(409,0.5)*math.cos(j2 + j3 - \
                        math.atan(20/3)) - 60*math.sin(j2)
        f3 = -self.zz + 60*math.cos(j1)*math.cos(j2) + \
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

    def trajectoryGen(self, plotFlag):
        th = np.linspace((2*np.pi/5)*self.claw, (2*np.pi/5)*(self.claw+1), self.points)
        idx = np.linspace(-self.points/2, self.points/2, self.points)

        self.x = self.radius * np.cos(th)
        self.y = self.radius * np.sin(th)
        self.z = (self.depth)/(self.points/2)**2 * idx**2 - self.depth + self.elevation

        if plotFlag is True:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(x,y,np.linspace(0, 0, self.points))
            plt.plot(x,y,z)
            ax.axis('equal')
            plt.show()

    def simulate(self, plotFlag):
        ptsTravel = np.linspace(0, self.points, math.floor(self.travelTime/self.cmdRate), endpoint=False)
        ptsReturn = np.linspace(0, self.points, math.floor(self.returnTime/self.cmdRate), endpoint=False)

        tList = []
        xList = []
        yList = []
        zList = []
        j1List = []
        j2List = []
        j3List = []
        start = timer()

        # Travel to endpoint
        for ii in range(len(ptsTravel)):
            start = timer()
            self.xx = self.x[np.int64(ptsTravel[ii])]
            self.yy = self.y[np.int64(ptsTravel[ii])]
            self.zz = self.elevation
            self.inverseKinematics()
            end = timer()

            tList.append(end-start)
            xList.append(self.xx)
            yList.append(self.yy)
            zList.append(self.zz)
            j1List.append(self.jointDesired1)
            j2List.append(self.jointDesired2)
            j3List.append(self.jointDesired3)

        # Reverse x and y
        self.x = self.x[::-1]
        self.y = self.y[::-1]

        # Travel back to start
        for ii in range(len(ptsReturn)):
            start = timer()
            self.xx = self.x[np.int64(ptsReturn[ii])]
            self.yy = self.y[np.int64(ptsReturn[ii])]
            self.zz = self.z[np.int64(ptsReturn[ii])]
            self.inverseKinematics()
            end = timer()

            tList.append(end-start)
            xList.append(self.xx)
            yList.append(self.yy)
            zList.append(self.zz)
            j1List.append(self.jointDesired1)
            j2List.append(self.jointDesired2)
            j3List.append(self.jointDesired3)

        if plotFlag is True:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(xList, yList, zList)
            ax.axis('equal')
            plt.show()

            plt.plot(tList)
            plt.show()

            plt.plot(j1List)
            plt.plot(j2List)
            plt.plot(j3List)
            plt.show()

def main():
    # claw, radius, points, elevation, depth, cmdRate, travelTime, returnTime
    claw0 = trajectoryKinematics(0, 230, 300, 250, 30, 0.1, 3, 1)
    claw0.trajectoryGen(plotFlag=False)

    claw0.simulate(plotFlag=True)

if __name__ == '__main__':
    main()
