function [Fc,Gc] = InnerLoopFeedback(var)

%-------------------------------------------------
% Inputs:
%   var: 12x1 quadrotor state vector

% Outputs:
%   Fc: control force
%   Gc: 3x1 control moments vector
%-------------------------------------------------

% Extract var Parameters
phi = var(4);
theta = var(5);
p = var(10);
q = var(11);
r = var(12);

% Constants
m = 0.068; %[kg]
g = 9.81; %[m/s^2]

% Gains
k1_lat = 0.001276;
k2_lat = 0.00232;
k1_lon = 0.0022;
k2_lon = 0.004;
k_spin = 0.004;

% Fc Calculation
Fc = -m*g;

% Gc Calclations
Lc = -k1_lat*p - k2_lat*phi;
Mc = -k1_lon*q - k2_lon*theta;
Nc = -k_spin*r;

Gc = [Lc;Mc;Nc];

end