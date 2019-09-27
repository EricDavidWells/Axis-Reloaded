import ETS3.*

p1 = [0, 0, 41];  % ground to first axis
p2 = [0, 56, 0];    
p3 = [0, 0, 60];
p4 = [0, -15, 100];

E = Tz(p1(3)) * Ry('q1') * Ty(p2(2)) * Rx('q2') * Tz(p3(3)) ...
    * Rx('q3') * Tz(p4(3)) * Ty(p4(2));

syms q1 q2 q3 real

TE = E.fkine([q1, q2, q3]);

% Power of Exponentials Method
theta1 = [0, 0, 0];
Tsb1 = Tsbgen5000(theta1);
Tas1 = [Rgamma(0), [0; 100; 0];[0, 0, 0], 1];
Tab1 = Tas1*Tsb1;

theta2 = [0, 0, 0];
Tsb2 = Tsbgen5000(theta2);
Tas2 = [Rgamma(72/180*pi), [-100*sin(72/180*pi); 100*cos(72/180*pi); 0];[0, 0, 0], 1];
Tab2 = Tas2*Tsb2;

theta5 = [0, 0, 0];
Tsb5 = Tsbgen5000(theta5);
Tas5 = [Rgamma(72*4/180*pi), [-100*sin(72*4/180*pi); 100*cos(72*4/180*pi); 0];[0, 0, 0], 1];
Tab5 = Tas5*Tsb5;
% 
% % Symbolic Inverse Kinematics
% syms t1 t2 t3 real
% theta1 = [t1, t2, t3];
% Tsb1 = Tsbgen5000(theta1);
% Tas1 = [Rgamma(0), [0; 100; 0];[0, 0, 0], 1];
% Tab1 = Tas1*Tsb1;
% 
% syms x y z real
% e1 = x == Tab1(1, 4);
% e2 = y == Tab1(2, 4);
% e3 = z == Tab1(3, 4);
% 
% S = solve([e1, e2, e3], [t1, t2, t3]);

% Numerical Inverse Kinematics
theta0 = [1, 1, 1];
pdes = [0; 100; 100];
fkin = @(theta)(AxisReloadedPoseCalc(0, theta));
fpos = @(T)T(1:3, 4);

err = @(theta) norm(fpos(fkin(theta)) - pdes);
options = optimset('TolFun', 0.1);
tic;
theta = fminsearch(err, theta0, options);
toc


