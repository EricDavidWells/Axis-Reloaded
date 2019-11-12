# Axis-Reloaded
This project uses 15 servo motors to articulate a robotic hand to rotate a platform, based on the popular project "Axis" made by Mark Sektrakian.  The hand will automatically correct the platform position if it starts to drift off center.  A video of the device in action can be seen here:


The control algorithm plan trajectories for each of the 3-DOF fingers and calculates the inverse kinematic solutions to each discritized path.  The Product of Exponentials forward kinematic solution is used, as well as a standard root solver for the inverse kinematics.  

# Hardware
A raspberry pi 3b+ runs the entire control algorithm.  The servo's used are all Lynx Motion HT1: http://www.lynxmotion.com/p-1127-lynxmotion-smart-servo-lss-motor-high-torque-ht1.aspx.


# Resources
Lynx Motion Communication Protocol

https://www.robotshop.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/


# Dependencies
Python 3.6
https://github.com/Lynxmotion/LSS_Library_Python
Scipy 1.3.2
Matplotlib 3.1.1
OpenCV 4.1.1.26
Imultils 0.5.3
