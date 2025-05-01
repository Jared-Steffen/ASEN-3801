function [Fc,Gc] = RotationDerivativeFeedback(var,m,g)

%-------------------------------------------------
% Inputs:
%   var: 12x1 quadrotor state vector
%   m: quadcopter mass
%   g: gravity

% Outputs:
%   Fc: control force
%   Gc: 3x1 control moments vector
%-------------------------------------------------

p = var(10);
q = var(11);
r = var(12);

k = 0.004; % given gain
Zc = -m*g;
Lc = k*-p;
Mc = k*-q;
Nc = k*-r;

Fc = Zc;
Gc = [Lc;Mc;Nc];

end