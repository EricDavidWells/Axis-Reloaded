import ETS3.*

p1 = [0, 0, 41];  % ground to first axis
p2 = [0, 56, 0];    
p3 = [0, 0, 60];
p4 = [0, -15, 100];

E = Tz(p1(3)) * Ry('q1') * Ty(p2(2)) * Rx('q2') * Tz(p3(3)) ...
    * Rx('q3') * Tz(p4(3)) * Ty(p4(2));

