function [var_dot,Lc,Mc,Nc] = QuadrotorEOMTranslation(t, var, g, m, I, nu, mu, mode)

%----------------------------------------------------------------
% Inputs:
%   var: 12x1 state vector
%   g: gravity
%   m: quadcopter mass
%   I: inertia matrix
%   nu: aerodynamic force coefficient
%   mu: aerodynamic moment coefficient
%   mode: string that is either 'Lateral' or 'Longitudinal'
%         *Note: Capitalization doesn't matter

% Outputs:
%   var_dot: 12xN state vector derivative
%   Lc: X control moment
%   Mc: Y control moment
%   Nc: Z control moment
%----------------------------------------------------------------

% Extract var/motor_forces/I data
phi = var(4);
theta = var(5);
psi = var(6);
uE = var(7);
vE = var(8);
wE = var(9);
inert_vel_b = [uE;vE;wE];
p = var(10);
q = var(11);
r = var(12);
ang_vel = [p;q;r];
Ix = I(1);
Iy = I(2);
Iz = I(3);

% Calculate Lc, Mc, Nc, Zc

M1 = 'Lateral';
M2 = 'Longitudinal';
Mcomp1 = strcmpi(M1,mode);
Mcomp2 = strcmpi(M2,mode);

if Mcomp1 == 1 && Mcomp2 == 0
    [Zc,Gc] = VelocityReferenceFeedbackLat(t,var);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
elseif Mcomp1 == 0 && Mcomp2 == 1
    [Zc,Gc] = VelocityReferenceFeedbackLon(t,var);
    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);
else
    disp('Invalid Mode Entered');
end

% Moments Calculations
moments = -mu * norm(ang_vel) * ang_vel;
L = moments(1);
M = moments(2);
N = moments(3);

% Aerodynamic Forces Calculation
aero_f = -nu * norm(inert_vel_b) * inert_vel_b;
X = aero_f(1);
Y = aero_f(2);
Z = aero_f(3);

% Trigoneometric Evaluated Angles
cPH = cos(phi);
cT = cos(theta);
cPS = cos(psi);
sPH = sin(phi);
sT = sin(theta);
sPS = sin(psi);
tT = tan(theta);
secT = sec(theta);

% Rotation Matrix
Rot321 = [cT*cPS,sPH*sT*cPS-cPH*sPS,cPH*sT*cPS+sPH*sPS;
             cT*sPS,sPH*sT*sPS+cPH*cPS,cPH*sT*sPS-sPH*cPS;
             -sT,sPH*cT,cPH*cT];

% Euler Angle Rates Matrix
EulMat = [1,sPH*tT,cPH*tT;
          0,cPH,-sPH
          0,sPH*secT,cPH*secT];

% Inerital Position Derivatives
inerital_pos_dot = Rot321 * [uE;vE;wE];

% Euler Angle Derivatives
euler_angle_rates = EulMat * ang_vel;

% Inertial Velocity in Body Derivatives
uE_dot = r*vE - q*wE - g*sT + X/m;
vE_dot = p*wE - r*uE + g*cT*sPH + Y/m;
wE_dot = q*uE - p*vE + g*cT*cPH + Z/m + (Zc/m);

% Roll Rate Derivatives
p_dot = (Iy - Iz)/Ix*q*r + L/Ix + Lc/Ix;
q_dot = (Iz - Ix)/Iy*p*r + M/Iy + Mc/Iy;
r_dot = (Ix - Iy)/Iz*p*q + N/Iz + Nc/Iz;

% Final State Derivative Vector
var_dot = [inerital_pos_dot;euler_angle_rates;uE_dot;vE_dot;wE_dot;p_dot;q_dot;r_dot];

end