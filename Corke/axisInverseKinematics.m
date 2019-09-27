import ETS3.*

% Axis of rotation and claw tip -> Starting ground to first axis
p1 = [0, 0, 41];
p2 = [0, 56, 0];
p3 = [0, 0, 60];
p4 = [0, -15, 100];

% Solve forward kinematics using Corke toolbox. E.teach to run GUI
E = Tz(p1(3)) * Ry('q1') * Ty(p2(2)) * Rx('q2') * Tz(p3(3)) ...
    * Rx('q3') * Tz(p4(3)) * Ty(p4(2));

syms q1 q2 q3 real
TE = E.fkine([q1, q2, q3]);

% Power of Exponentials Method -> Numerical
%    Tsbgen5000([servoAngle1, servoAngle2, servoAngle3]) -> Claw tip pose in claw frame
%    Rgamma(gamma) -> Calculates yaw rotation matrix
%    Tsb -> Claw tip pose in claw frame
%    Tab -> Center of disk to claw
%    Tab -> Center of disk to end of claw pose
for ii = 0:4
  Tsb = Tsbgen5000([0, 0, 0]);
  Tas = [Rgamma((72*ii)/180*pi), [-100*sind(72*ii); 100*cosd(72*ii); 0]; [0, 0, 0], 1];
  Tab{ii+1} = Tas*Tsb;
end

% % Power of Exponentials Method (forward kinematics)-> Symbolic
% syms j1 j2 j3 real
% Tsb = Tsbgen5000([j1, j2, j3]);
% Tas = [Rgamma(0), [-100*sind(0); 100*cosd(0); 0]; [0, 0, 0], 1];
% Tab = simplify(Tas*Tsb);
%
% % Symbolic inverse kinematics
% syms x y z real
% e1 = x == Tab(1, 4);
% e2 = y == Tab(2, 4);
% e3 = z == Tab(3, 4);
%
% S = solve([e1 e2 e3], [j1 j2 j3])

% Numerical Inverse Kinematics
theta0 = [0, 0, 0];                                % initial guess
pdes = [90; 100; 150];                             % desired end effector position
fkin = @(theta)(AxisReloadedPoseCalc(0, theta));   % function to compute pose of finger 0
fpos = @(T)T(1:3, 4);                              % function to extract position from pose
err = @(theta) norm(fpos(fkin(theta)) - pdes);     % error between guess and desired

% fminsearch method
options = optimset('TolFun', 0.001);
tic;
[theta, error] = fminsearch(err, theta0, options)
toc

% unconstrained fminunc method
options = optimoptions('fminunc', 'ObjectiveLimit', 0.001);
tic;
[theta, error] = fminunc(err, theta0, options)
toc

% constrained fmincon method
options = optimoptions('fmincon', 'ObjectiveLimit', 0.001);
tic;
[theta, error] = fmincon(err, theta0, [],[],[],[],[-pi,-pi,-pi], [pi,pi,pi],[], options)
toc
