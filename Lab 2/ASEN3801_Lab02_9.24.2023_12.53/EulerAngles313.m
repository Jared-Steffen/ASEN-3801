function [attitude] = EulerAngles313(RotMat)
%   EULERANGLES321 | This function is used to extract the euler rotation
%   angles present in a 3-2-1 euler rotation matrix
%   Input: Euler 3-2-1 Rotation Matrix
%   Output: 3 x 1 vector of phi-theta-psi euler angles

    % Calulating psi, theta, and phi based on the Euler 3-1-3 formula by manipulating together certain position values in the inputted rotation matrix

    LMGA = atan2d(RotMat(1,3),RotMat(2,3));
    i = acosd(RotMat(3,3));
    UMGA = atan2d(RotMat(3,1),-RotMat(3,2));

    attitude = [LMGA; i; UMGA]; % Output a vector of the 3-1-3 euler angles

end
