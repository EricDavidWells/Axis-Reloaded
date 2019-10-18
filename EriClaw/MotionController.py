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
import pickle
from scipy.interpolate import RegularGridInterpolator as rgi
import threading
import platform


class MotionController:
    mode = 0
    correction = np.array([0, 0, 0])

    def run(self):
        global mode 
        global correction
        global state
        state = 0
        mode = 0
        correction = np.array([0, 0, 0])
        print("Initializing Motion Controller")
        t = threading.Thread(target=self.run_thread)
        t.start()
    def getMCState(self):
        return state
    def run_thread(self):
        global state
        # Initialize serial bus
        if platform.system() == "Windows":
            # Eric
            comport = 'COM7'
            # # Riley
            # comport = 'COM4'
        else: 
            comport = "/dev/ttyUSB0"
        baudrate = lssc.LSS_DefaultBaud
        lss.initBus(comport, baudrate)

        fingers = [Finger([lss.LSS(11), lss.LSS(12), lss.LSS(13)], 0),
                   Finger([lss.LSS(21), lss.LSS(22), lss.LSS(23)], 0),
                   Finger([lss.LSS(31), lss.LSS(32), lss.LSS(33)], 0),
                   Finger([lss.LSS(41), lss.LSS(42), lss.LSS(43)], 0),
                   Finger([lss.LSS(51), lss.LSS(52), lss.LSS(53)], 0)]

        fangles = [0, 72 / 180 * pi, 72 * 2 / 180 * pi, 72 * 3 / 180 * pi, 72 * 4 / 180 * pi]
        radius = 69
        hand = Hand(fingers, fangles, radius)
        hand.reset()

        x, y, z, data0, data1, data2 = pickle.load(open("IK_pointcloud.1.p", "rb"))

        # Local Point Cloud Interpolation Generation

        # finger = Finger([lss.LSS(69)], 0)
        # gridpoints = np.array([100, 100, 50])
        # gridstart = np.array([-100, -100, 100])
        # gridend = np.array([100, 100, 220])
        # x, y, z, pointcloud = TrajectoryGen.cubicpointcloudgen(gridstart, gridend, gridpoints, plotFlag=True)
        # ikinehelper = lambda pd, guess: finger.ikine(pd, guess)
        # data0, data1, data2 = TrajectoryGen.inversepointcloud(x, y, z, ikinehelper)
        # pickle.dump([x, y, z, data0, data1, data2], open("IK_pointcloud.p", "wb"))

        # Generate global interpolation functions for each finger
        interpfun = TrajectoryGen.interpfungen(x, y, z, data0, data1, data2)
        interpfunglobal = []
        offset = [0, 0, 0, 0, 0]
        for i in range(0, len(hand.fingers)):
            temp = TrajectoryGen.interpfungenglobal(np.linalg.inv(hand.Tas[i]), interpfun, offset[i])
            interpfunglobal.append(temp)

        # Create Spin Trajectories
        points = 2000
        trajoffset = 2 * pi / 180
        spinr = 125
        spinh = 175
        spindepth = 30
        spintraj = []
        spinitraj = []
        spinitraj2 = []
        for i in range(0, len(hand.fingers)):
            spintraj.append(TrajectoryGen.spin(spinr, spinh, spindepth, points, 4, i, trajoffset, plotflag=False))
            # ikinehelper = lambda pd, guess: hand.ikine(pd, guess, fingernum=i)
            # spinitraj.append(TrajectoryGen.inverse(traj[i], ikinehelper, plotFlag=1))
            spinitraj2.append(TrajectoryGen.inverse(spintraj[i], interpfunglobal[i], plotFlag=False))

        # _ = input("ready to start: ")
        for j in range(0, 5):
            hand.move(spinitraj2[j][:, 0], j)
            # hand.move(np.array([0, 0, 0]), j)
        time.sleep(3)
        print("Motion Controller Ready. Place plate on claw and press enter to continue.")
        while True:
            # Modes:
            # 0 = pause
            # 1 = spin
            # 2 = linear correction
            # 3 = shut off motors

            if mode == 1:
                state = 1
                for i in range(0, max(spinitraj2[0].shape)):
                    for j in range(0, 5):

                        while mode == 0:
                            time.sleep(0.1)

                        if mode == 4:
                            break

                        hand.move(spinitraj2[j][:, i], j)

            if mode == 2:
                # Center
                # Move to center finger positions one at a time

                state = 2
                lintraj1 = []
                ilintraj1 = []
                epostracker = [0, 0, 0, 0, 0]
                for i in range(0, 5):

                    linpoints1 = 750
                    spos = spintraj[i][:, 0]
                    eposl = np.array([0, spinr-69, spinh])
                    T = hand.Tas[i]
                    temp = np.array(eposl).reshape(3, 1)
                    posl = np.vstack((temp, 1))
                    temp2 = T @ posl
                    epos = temp2[0:3]
                    epos = epos.reshape(3)
                    epostracker[i] = epos
                    if i == 4:
                        lintraj1.append(TrajectoryGen.linear(spos, epos, linpoints1, plotflag=False))
                        ilintraj1.append(TrajectoryGen.inverse(lintraj1[i], interpfunglobal[i], plotFlag=False))
                    else:
                        lintraj1.append(TrajectoryGen.linear_parabolicz(spos, epos, linpoints1, 30, plotflag=False))
                        ilintraj1.append(TrajectoryGen.inverse(lintraj1[i], interpfunglobal[i], plotFlag=False))

                for i in range(4, -1, -1):
                    for j in range(0, max(lintraj1[0].shape)):
                        hand.move(ilintraj1[i][:, j], i)

                        while mode == 0:
                            time.sleep(0.1)

                        if mode == 4:
                            break

                # Move all fingers in target direction
                lintraj2 = []
                ilintraj2 = []
                for i in range(0, len(hand.fingers)):

                    spos = epostracker[i]
                    epos = spos - correction
                    epostracker[i] = epos
                    linpoints2 = 250
                    lintraj2.append(TrajectoryGen.linear(spos, epos, linpoints2, plotflag=False))
                    ilintraj2.append(TrajectoryGen.inverse(lintraj2[i], interpfunglobal[i], plotFlag=False))
                for i in range(0, max(lintraj2[0].shape)):
                    for j in range(0, 5):
                        hand.move(ilintraj2[j][:, i], j)

                        while mode == 0:
                            time.sleep(0.1)

                        if mode == 4:
                            break

                # Move all fingers back to starting spin position
                lintraj3 = []
                ilintraj3 = []
                for i in range(0, len(hand.fingers)):
                    spos = epostracker[i]
                    epos = spintraj[i][:, 0]
                    epos = epos.reshape(3)
                    linpoints3 = 750
                    lindepth = 30
                    if i == 4:
                        lintraj3.append(TrajectoryGen.linear(spos, epos, linpoints3, plotflag=False))
                        ilintraj3.append(TrajectoryGen.inverse(lintraj3[i], interpfunglobal[i], plotFlag=False))
                    else:
                        lintraj3.append(TrajectoryGen.linear_parabolicz(spos, epos, linpoints3, lindepth, plotflag=False))
                        ilintraj3.append(TrajectoryGen.inverse(lintraj3[i], interpfunglobal[i], plotFlag=False))

                for i in range(0, 5):
                    for j in range(0, max(lintraj3[0].shape)):
                        hand.move(ilintraj3[i][:, j], i)

                        while mode == 0:
                            time.sleep(0.1)

                        if mode == 4:
                            break

            if mode == 3:
                state = 3
                for i in range(0, len(hand.fingers)):
                    hand.fingers[i].limp()

                lss.closeBus()
                return

        pass

    def set_mode(self, mode_):
        global mode
        mode = mode_

    def set_correction(self, correction_):
        global correction
        correction = correction_


class Hand:

    def __init__(self, fingers_, fangles_, radius_):
        self.fingers = fingers_
        self.fangles = fangles_
        self.radius = radius_
        self.Tas = [self.Tasgen(fnum, fangle) for fnum, fangle in enumerate(fangles_)]
        self.J = [fi.J for fi in self.fingers]
        self.pos = [self.fkine(fi.J, i) for i, fi in enumerate(self.fingers)]

    def Tasgen(self, fingernum, fangle):
        r = self.radius
        R = Rgamma(fangle)
        p = np.array([-r * sin(fangle), r * cos(fangle), 0]).reshape(3, 1)
        temp = np.hstack((R, p))
        Tas = np.vstack((temp, [0, 0, 0, 1]))
        return Tas

    def fkine(self, theta, fnum):
        Tsb = self.fingers[fnum].Tsbgen(theta)
        Tas = self.Tas[fnum]
        Tab = Tas@Tsb
        pos = Tab[0:3, 3]
        return pos

    def ikine(self, posDesired, guess, fingernum):
        iJ = root(self.ikine_err, guess, args=(posDesired, fingernum), tol=0.001)
        return iJ.x, iJ.fun

    def ikine_err(self, theta, posDesired, fingernum):
        e = self.fkine(theta, fingernum) - np.array(posDesired)
        return e[0], e[1], e[2]

    def reset(self, waittime=3):
        for fi in self.fingers:
            fi.reset()

    def move(self, theta, fingernum):
        self.fingers[fingernum].move(theta)
        self.J[fingernum] = self.fingers[fingernum].J
        self.pos[fingernum] = self.fkine(self.J[fingernum], fingernum)

    def moveall(self, theta):
        for fi in self.fingers:
            fi.move(theta)
        self.J = [fi.J for fi in self.fingers]
        self.pos = [self.fkine(fi.J, i) for i, fi in enumerate(self.fingers)]


class Finger:
    def __init__(self, servoarray, zoffset):
        """

        :param J0: initial theta vector of finger
        """
        self.servos = servoarray

        # All kinematic constants defined
        self.Tsb0 = np.array([[1,0,0,0],
                             [0,1,0,35.7],
                             [0,0,1,202.9 + zoffset],
                             [0,0,0,1]])
        self.w1 = np.array([0, -1, 0]).reshape(3,1)
        self.p1 = np.array([0, 0, 43.1]).reshape(3,1)
        self.w2 = np.array([-1, 0, 0]).reshape(3,1)
        self.p2 = np.array([0, 55.15, 43.1]).reshape(3,1)
        self.w3 = np.array([1, 0, 0]).reshape(3,1)
        self.p3 = np.array([0, 55.15, 104.5]).reshape(3,1)
        self.Jlim = np.array([[-90, 90], [-30, 110], [-80, 120]])
        self.J = np.array([0, 0, 0])
        self.pos = self.fkine(self.J)

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
        itheta = root(self.ikine_err, guess, args=posDesired, tol=0.001)
        return itheta.x, itheta.fun

    def ikine_err(self, theta, posDesired):
        e = self.fkine(theta) - np.array(posDesired)
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

    def spaceJacobian(self, theta):
        X1 = self.exiGen(self.w1, self.p1)
        eX1 = self.eXgen(self.p1, self.w1, theta[0])
        Js1 = X1

        X2 = self.exiGen(self.w2, self.p2)
        eX2 = self.eXgen(self.p2, self.w2, theta[1])
        Js2 = Adjoint(eX1)@X2

        X3 = self.exiGen(self.w3, self.p3)
        Js3 = Adjoint(eX1@eX2)@X3

        temp = np.hstack((Js1, Js2))
        Js = np.hstack((temp, Js3))
        return Js

    def bodyJacobian(self, theta):
        Js = self.spaceJacobian(theta)
        Tsb = self.Tsbgen(theta)
        Tbs = np.linalg.inv(Tsb)
        Jb = Adjoint(Tbs)@Js
        return Jb

    def ikine_jacobian(self, Tsd, guess, tol):

        Tsb = self.Tsbgen(guess)
        Tbs = np.linalg.inv(Tsb)
        temp = Tbs@Tsd
        Vb = np.log(Tbs@Tsd)


        while np.linalg.norm(Vb) > tol:
            Tsb = self.Tsbgen(guess)
            Tbs = np.linalg.inv(Tsb)
            Vb = np.log(Tbs@Tsd)
            guess = guess + self.bodyJacobian(guess)@Vb

    def limp(self):
        for i in range(0, len(self.servos)):
            self.servos[i].limp()
        return

    def exiGen(self, w, p):
        exi = np.vstack((self.w1, np.cross(-self.w1.reshape(1,3), self.p1.reshape(1,3)).reshape(3,1)))
        return exi

    def move(self, J):
        Jdeg = J*180/pi
        for i in range(0, len(self.servos)):
            Jdeg[i] = np.clip(Jdeg[i], self.Jlim[i, 0], self.Jlim[i, 1])
            value = int(Jdeg[i]*10)
            self.servos[i].move(value)
        self.J = J
        self.pos = self.fkine(J)

    def reset(self, waittime=3):
        for i in range(0, len(self.servos)):
            self.servos[i].reset()
        time.sleep(waittime)
        for i in range(0, len(self.servos)):
            self.servos[i].setMaxSpeed(20)
            self.servos[i].setMotionControlEnabled(0)
            # self.servos[i].setAngularAcceleration(10)
            # self.servos[i].setAngularDeceleration(10)
            # self.servos[i].setAngularStiffness(6)
            # self.servos[i].setAngularHoldingStiffness(4)


class TrajectoryGen:
    def __init__(self):
        pass

    def circle(radius, height, center, points):
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

    def inverse(traj, fun, plotFlag = False):
        """
        Creates the inverse kinematics from the forward kinematics 'traj' based on the inverse kinematics function fun
        :param traj: numpy array of shape 3xN.  [[x1, x2, ...., xn], [y1, y2, ...., yn], [z1, z2, ...., zn]
        :param fun: function that computes the error of the forward kinematics guessed joint angles and the desired position
                    from the trajectory.  Must take as parameters a numpy array of [xd, yd, zd] and a guess of the correct
                    inverse kinematics of [th1, th2, th3].  Must return the error between the actual position and position
                    from the forward kinematics of the guess.
        :return: itraj: numpy array of shape 3xN containing joint angles
        """
        points = max(traj.shape)
        itraj = np.zeros(traj.shape)
        p = np.arange(0, points, 1)
        for i in range(0, points):
            iJ, err = fun(traj[:, i], itraj[:, i - 1])
            itraj[:, i] = iJ

        if plotFlag == True:
            fig = plt.figure()
            plt.plot(p, itraj[0, :])
            plt.plot(p, itraj[1, :])
            plt.plot(p, itraj[2, :])
            plt.xlabel("i")
            plt.ylabel("theta (rads)")
            plt.legend(["theta1", "theta2", "theta3"])
            plt.show()

        return itraj

    def spin(radius, height, depth, points, returnspeed, fingernum, offset, plotflag=False):
        th = np.linspace((2*pi/5)*fingernum + offset, (2*pi/5)*(fingernum+1) - offset, points)-pi/5
        returnpoints = points/returnspeed
        idx = np.linspace(-returnpoints/2, returnpoints/2, returnpoints)

        x = -radius * np.sin(th)
        y = radius * np.cos(th)
        z1 = np.ones(x.shape)*height

        z2 = depth/((points/returnspeed/2)**2)*(idx**2)+height-depth

        xf = np.append(x, np.flip(x)[::returnspeed])
        yf = np.append(y, np.flip(y)[::returnspeed])
        zf = np.append(z1, z2)
        pLength= max(xf.shape)
        traj = np.array([np.roll(xf,round(points*fingernum - points/8)),
                         np.roll(yf,round(points*fingernum - points/8)),
                         np.roll(zf,round(points*fingernum - points/8))])

        if plotflag is True:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(xf, yf, zf)
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
            ax.axis('equal')
            plt.show()

        return traj

    def linear(startpos, endpos, points, plotflag=False):
        traj = np.linspace(startpos, endpos, points).transpose()

        if plotflag is True:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(traj[0,:], traj[1,:], traj[2,:])
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
            ax.axis('equal')
            plt.show()
        return traj

    def linear_parabolicz(startpos, endpos, points, depth, plotflag=True):
        traj = np.linspace(startpos, endpos, points).transpose()
        idx = np.linspace(-points/2, points/2, points)
        z = depth / ((points/2)**2) * (idx**2) + startpos[2] - depth
        traj[2,:] = z

        if plotflag is True:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(traj[0,:], traj[1,:], traj[2,:])
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
            ax.axis('equal')
            plt.show()

        return traj

    def cubicpointcloudgen(gridstart, gridend, gridpoints, plotFlag=False):
        x = np.linspace(gridstart[0], gridend[0], gridpoints[0])
        y = np.linspace(gridstart[1], gridend[1], gridpoints[1])
        z = np.linspace(gridstart[2], gridend[2], gridpoints[2])
        xmesh, ymesh, zmesh = np.meshgrid(x, y, z)

        if plotFlag == True:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(xmesh.flatten(), ymesh.flatten(), zmesh.flatten(), 'o')
            plt.show()

        pointcloud = np.array([xmesh.flatten(), ymesh.flatten(), zmesh.flatten()])
        return x, y, z, pointcloud

    def inversepointcloud(x, y, z, fun):
        gridpoints = [x.size, y.size, z.size]
        data0 = np.zeros(gridpoints)
        data1 = np.zeros(gridpoints)
        data2 = np.zeros(gridpoints)

        for i in range(0, gridpoints[0]):
            for j in range(0, gridpoints[1]):
                for k in range(0, gridpoints[2]):
                    iJ, err = fun([x[i], y[j], z[k]], [0, 0, 0])
                    data0[i, j, k] = iJ[0]
                    data1[i, j, k] = iJ[1]
                    data2[i, j, k] = iJ[2]

        return data0, data1, data2

    def interpfungen(x, y, z, data0, data1, data2):
        fn0 = rgi((x, y, z), data0)
        fn1 = rgi((x, y, z), data1)
        fn2 = rgi((x, y, z), data2)

        def fun(pos, guess):
            postuple = (pos[0], pos[1], pos[2])
            theta = [fn0(postuple), fn1(postuple), fn2(postuple)]
            err = False
            return theta, err

        return fun

    def interpfungenglobal(T, fun, offset):
        def fung(pos, guess):
            pos[2] = pos[2] + offset
            temp = np.array(pos).reshape(3,1)
            posg = np.vstack((temp, 1))
            temp2 = T@posg
            posl = temp2[0:3]
            return fun(posl, guess)
        return fung


def skew(v):
    a = v[0,0]
    b = v[1,0]
    c = v[2,0]
    s = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
    return s


def Rgamma(gamma):
    R = np.array([[cos(gamma), -sin(gamma), 0],
        [sin(gamma),   cos(gamma), 0],
        [0,            0,          1]])
    return R


def Adjoint(T):
    R = T[0:3, 0:3]
    p = T[0:3,3].reshape(3,1)
    temp = np.hstack((R, np.zeros([3,3])))
    temp2 = np.hstack((skew(p)@R, R))
    Ad = np.vstack((temp, temp2))
    return Ad

