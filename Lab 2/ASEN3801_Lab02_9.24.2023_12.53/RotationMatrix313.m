function [R313] = RotationMatrix313(attitude)
%   ROTATIONMATRIX313 | This function is used to determine the euler 3-1-3 rotation matrix
%   given a set of inputted euler angles
%   Inputs = 3 x 1 vector of LowercaseOmega-i-UppercaseOmega angles
%   Output = Euler 3-1-3 Rotation Matrix
    
    LMGA = attitude(1); % Euler Angle Lowercase Omega
    i = attitude(2); % Euler Angle i
    UMGA = attitude(3); % Euler Angle Uppercase Omega
    
    % [Inertial to Body Frame] Plugging into formula
    R313 = [ cosd(UMGA) .* cosd(LMGA) - sind(UMGA) .* sind(LMGA) .* cosd(i), -sind(UMGA) .* cosd(LMGA) - cosd(UMGA) .* sind(LMGA) .* cosd(i), sind(i) .* sind(LMGA);
             cosd(UMGA) .* sind(LMGA) + sind(UMGA) .* cosd(i) .* cosd(LMGA), -sind(UMGA) .* sind(LMGA) + cosd(UMGA) .* cosd(i) .* cosd(LMGA), -sind(i) .* cosd(LMGA);
             sind(UMGA) .* sind(i), cosd(UMGA) .* sind(i), cosd(i)]';
    
end

