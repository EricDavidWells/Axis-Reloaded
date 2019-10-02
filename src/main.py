import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from timeit import default_timer as timer
from scipy.optimize import root
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

    def ikine(self, posDesired, guess):
        """
        Compute inverse kinematics joint angles of desired position of end effector relative to finger base
        """
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
    def readyToMove(self):
        ready = True
        # for i in range(0, len(self.servos)):
            #
            # stat = self.servos[i].getPosition()
            # while stat is None:
            #     stat= self.servos[i].getPosition()
        #         time.sleep(0.0001)
        #

            # if ("6" not in stat) and ("5" not in stat):
            #     ready = False
            #     break
        return ready

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
        for i in range(0, len(self.servos)):
            self.servos[i].setMaxSpeed(30)
            # self.servos[i].setAngularAcceleration(10)
            # self.servos[i].setAngularDeceleration(10)
            # self.servos[i].setAngularStiffness(6)
            # self.servos[i].setAngularHoldingStiffness(4)
            self.servos[i].setMotionControlEnabled(0)


def circleTrajectoryGen(radius, height, center, points):
    th = np.linspace(0, 2*pi, points)
    x = radius*np.cos(th) + center[0]
    y = radius*np.sin(th) + center[1]
    z = np.ones(th.shape)*height

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.plot(x, y, z)
    ax.axis('equal')
    plt.show()

    return np.array([x, y, z])


def inverseTracjectoryGen(traj, fun):
    points = max(traj.shape)
    itraj = np.zeros(traj.shape)
    p = np.arange(0, points, 1)
    for i in range(0, points):
        iJ, err = fun(traj[:, i], itraj[:, i - 1])
        itraj[:, i] = iJ

    fig = plt.figure()
    plt.plot(p, itraj[0, :])
    plt.plot(p, itraj[1, :])
    plt.plot(p, itraj[2, :])
    plt.show()

    return itraj


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

        self.x = 0
        self.y = 0
        self.z = 0


    def trajectoryGen(self, plotFlag):
        th = np.linspace((2*np.pi/5)*self.claw, (2*np.pi/5)*(self.claw+1), self.points)
        idx = np.linspace(-self.points/2, self.points/2, self.points)

        self.x = self.radius * np.cos(th)
        self.y = self.radius * np.sin(th)
        self.z = (self.depth)/(self.points/2)**2 * idx**2 - self.depth + self.elevation

        if plotFlag is True:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(self.x,self.y,np.linspace(0, 0, self.points))
            plt.plot(self.x,self.y,self.z)
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

            self.jointActual1 = self.jointDesired1
            self.jointActual2 = self.jointDesired2
            self.jointActual3 = self.jointDesired3


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


def skew(v):
        a, b, c = v[0,0], v[1,0], v[2,0]
        s = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
        return s


def main():
    # initialize servos
    comport = 'COM6'
    servoID1 = 21
    servoID2 = 22
    servoID3 = 23
    baudrate = lssc.LSS_DefaultBaud
    lss.initBus(comport, baudrate)
    myLSS1 = lss.LSS(servoID1)
    myLSS2 = lss.LSS(servoID2)
    myLSS3 = lss.LSS(servoID3)

    # Create finger object
    finger = Finger([myLSS1, myLSS2, myLSS3])
    finger.reset()

    # Move to desired starting position
    pos = [0, 10, 150]
    iJ, err = finger.ikine(pos, finger.J)
    finger.move(iJ)
    pos = finger.fkine(iJ)
    print(iJ, err, pos)

    # generate circle trajectory
    points = 3600
    traj = circleTrajectoryGen(50, 175, [0, 30], points)
    itraj = inverseTracjectoryGen(traj, finger.ikine)

    # Loop through trajectory
    while True:
        for i in range(0, points):
            # start = time.time()
            if finger.readyToMove():
                finger.move(itraj[:, i])
            else:
                i -= 1
            time.sleep(0.005)
            # print(time.time()-start)
            # print(finger.servos[1].getCurrent())


if __name__ == '__main__':
    main()
