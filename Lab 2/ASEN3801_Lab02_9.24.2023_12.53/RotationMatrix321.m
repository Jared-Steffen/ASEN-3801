function [R321] = RotationMatrix321(attitude)
%   ROTATIONMATRIX321 | This function is used to determine the euler 3-2-1 rotation matrix
%   given a set of inputted euler angles
%   Inputs = 3 x 1 vector of phi-theta-psi angles
%   Output = Euler 3-2-1 Rotation Matrix
    
    phi = attitude(1); % Euler Angle phi
    theta = attitude(2); % Euler Angle theta
    psi = attitude(3); % Euler Angle psi
    
    % [Inertial to Body Frame] Plugging into formula
    R321 = [ cosd(phi) .* cosd(psi), sind(phi) .* sind(theta) .* cosd(psi) - cosd(phi) .* sind(psi), cosd(phi) .* sind(theta) .* cosd(psi) + sind(phi) .* sind(psi); 
             cosd(theta) .* sind(psi), sind(phi) .* sind(theta) .* sind(psi) + cosd(phi) .* cosd(psi), cosd(phi) .* sind(theta) .* sind(psi) - sind(phi) .* cosd(psi); 
             -sind(theta), sind(phi) .* cosd(theta), cosd(phi) .* cosd(theta)]';

end

