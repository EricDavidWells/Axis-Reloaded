function Tsb = Tsbgen5000(theta)
  % Starting pose of end effector relative to initial servo
  Tsb0 = [1, 0, 0, 0;
          0, 1, 0, 41;
          0, 0, 1, 201;
          0, 0, 0, 1];

  % Revolute joint 1
  w1 = [0, 1, 0]';
  p1 = [0, 0, 41]';
  eX1 = expogen4000(p1, w1, theta(1));

  % Revolute joint 2
  w2 = [1, 0, 0]';
  p2 = [0, 56, 41]';
  eX2 = expogen4000(p2, w2, theta(2));

  % Revolute joint 3
  w3 = [1, 0, 0]';
  p3 = [0, 56, 101]';
  eX3 = expogen4000(p3, w3, theta(3));

  % Calculate Tsb in claw frame
  Tsb = eX1*eX2*eX3*Tsb0;
end
