function [Tab, Rab, Pab] = AxisReloadedPoseCalc(fingernum, theta)

    Tsb = Tsbgen5000(theta);
    Tas = [Rgamma(72*fingernum/180*pi), [0; 100; 0]; [0, 0, 0], 1];
    Tab = Tas*Tsb;
    Rab = Tab(1:3, 1:3);
    Pab = Tab(1:3, 4);

end
