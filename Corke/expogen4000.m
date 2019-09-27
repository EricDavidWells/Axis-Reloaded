function eX = expogen4000(p, w, theta)
  % Calculate individual matrix exponential terms
  z = cross(-w, p);
  eS = eye(3) + skew(w)*sin(theta) + skew(w)*skew(w)*(1-cos(theta));
  G = eye(3)*theta + skew(w)*(1-cos(theta))+skew(w)*skew(w)*(theta-sin(theta));
  eX = [eS,      G*z;
       [0, 0, 0], 1];
end
