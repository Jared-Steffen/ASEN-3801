function [attitude] = EulerAngles321(RotMat)
%   EULERANGLES321 | This function is used to extract the euler rotation
%   angles present in a 3-2-1 euler rotation matrix
%   Input: Euler 3-2-1 Rotation Matrix
%   Output: 3 x 1 vector of phi-theta-psi euler angles

    % Calulating psi, theta, and phi based on the Euler 3-2-1 formula by manipulating together certain position values in the inputted rotation matrix

    psi = atan2d(RotMat(1,2),RotMat(1,1)); 
    theta = asind(RotMat(1,3));
    phi = atan2d(RotMat(2,3),RotMat(3,3));

    attitude = [phi; theta; psi]; % Output a vector of the 3-2-1 euler angles

end

