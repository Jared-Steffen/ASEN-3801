function [Fc,Gc] = VelocityReferenceFeedbackLon(t,var)

%-------------------------------------------------
% Inputs:
%   t: time vector
%   var: 12x1 quadrotor state vector

% Outputs:
%   Fc: control force
%   Gc: 3x1 control moments vector
%-------------------------------------------------

% Constants
m = 10; %[kg]
g = 9.81; %[m/s^2]

% Gain Values
k1_lat = 0.001276;
k2_lat = 0.00232;
k3_lat = 1.102E-4;
k1_lon = 0.001584;
k2_lon = 0.00288;
k3_lon = -1.368E-4;
k_spin = 0.004;

% var Values Needed
phi = var(4);
theta = var(5);
u = var(7);
v = var(8);
p = var(10);
q = var(11);
r = var(12);

% Determine vr, ur From Time
if t <= 2
    vr = 0;
    ur = 0.5;
else
    vr = 0;
    ur = 0;
end

% Calculate Lc, Mc, Nc
Lc = -k1_lat*p - k2_lat*phi + k3_lat*(vr - v);
Mc = -k1_lon*q - k2_lon*theta + k3_lon*(ur - u);
Nc = -k_spin*r;

% Fc and Gc
Fc = -m*g;
Gc = [Lc;Mc;Nc];

end