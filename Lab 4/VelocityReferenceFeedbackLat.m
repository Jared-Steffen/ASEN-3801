function [Fc,Gc] = VelocityReferenceFeedbackLat(t,var)

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
Ix = 0.17215;
Iy = 0.19460;
Iz = 0.20743;

% Gain Values
k1_lat = 22*Ix;
k2_lat = 40*Ix;
k3_lat = 0.3271;
k1_lon = 22*Iy;
k2_lon = 40*Iy;
k3_lon = -1.368;
k_spin = 0.004;

% var Values Needed
x = var(1);
y = var(2);
phi = var(4);
theta = var(5);
u = var(7);
v = var(8);
p = var(10);
q = var(11);
r = var(12);

% Determine vr, ur From t
if t <= 30
    vr = 4;
    ur = 0;
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