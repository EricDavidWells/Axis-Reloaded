function R = Rgamma(gamma)
    % Yaw rotation matrix
    R = [cos(gamma), -sin(gamma), 0;
        sin(gamma),   cos(gamma), 0;
        0,            0,          1];
end
