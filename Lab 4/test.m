clc; clear; close all

% Quadrotor Constants
m = 10; %[kg]
d = 0.460; %[m]
km = 0.0024; %[Nm/N]
Ix = 5.8 * 10^-3; %[kgm^2]
Iy = 7.2 * 10^-3; %[kgm^2]
Iz = 1.0 * 10^-2; %[kgm^2]
nu = 5.0 * 10^-3; %[N/(m/s)^2]
mu = 5.0 * 10^-6; %[Nm/(rad/s)^2]
g = 9.81; %[m/s^2]

% Inertia Matrix
I = [Ix;Iy;Iz];

% Initial Conditions
var_a = [0;0;0;0;0;0;0;0;0;0;0;0];

% Time
tspan = [0 3.15]; %[s]

% Motor Forces/Figure Vectors
W = m*g; %[N]
Zc_a = -W;
motor_forces_a = [-Zc_a/2; -Zc_a/2; -Zc_a/2; -Zc_a/2];
controls_a = [Zc_a,0,0,0];
Gc_a = [0;0;0];
fig_a = 1:6;

% ode45 Call
[ta,var_dot_a] = ode45(@(tspan,var_a) QuadrotorEOM(tspan,var_a,g,m,I,d,km,nu,mu,motor_forces_a),tspan,var_a);

% PlotAircraftSim Call
PlotAircraftSim(ta,var_dot_a,controls_a,fig_a,'-b')
