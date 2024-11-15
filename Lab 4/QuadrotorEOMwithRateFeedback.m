function [var_dot,Lc,Mc,Nc] = QuadrotorEOMwithRateFeedback(~,var,g,m,I,nu,mu)

%----------------------------------------------------------------
% Inputs:
%   var: 12x1 state vector
%   g: gravity
%   m: quadcopter mass
%   I: inertia matrix
%   nu: aerodynamic force coefficient
%   mu: aerodynamic moment coefficient

% Outputs:
%   var_dot: 12xN state vector derivative
%   Lc: X control moment
%   Mc: Y control moment
%   Nc: Z control moment
%----------------------------------------------------------------

% Extract var/motor_forces/I Data
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
airspeed = norm(inert_vel_b);
omega_mag = norm(ang_vel);


% Angular Control
[Fc,Gc] = RotationDerivativeFeedback(var,m,g);
Lc = Gc(1);
Mc = Gc(2);
Nc = Gc(3);

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

% Inerital Position Derivatives
inerital_pos_dot = Rot321 * inert_vel_b;


% Euler Angle Rates Matrix
EulMat = [1,sPH*tT,cPH*tT;
          0,cPH,-sPH
          0,sPH*secT,cPH*secT];
         
% Euler Angle Derivatives
euler_angle_rates = EulMat * ang_vel;

% Calculate velocity dynamics
vel_dot1 =    [ang_vel(3)*inert_vel_b(2)-ang_vel(2)*inert_vel_b(3);
                 ang_vel(1)*inert_vel_b(3)-ang_vel(3)*inert_vel_b(1);
                 ang_vel(2)*inert_vel_b(1)-ang_vel(1)*inert_vel_b(2)];
       
vel_dot2 =    g * [-sT;
                    cT*sPH;
                    cT*cPH];
             
vel_dot3 =    -nu*airspeed/m * inert_vel_b;
             
vel_dot4 =    [0;0;Fc/m];

vel_dot = vel_dot1 + vel_dot2 + vel_dot3 + vel_dot4;

% Roll Rate Derivatives
ang_vel_dot1 =    [(Iy-Iz)*ang_vel(2)*ang_vel(3)/Ix;
                   (Iz-Ix)*ang_vel(1)*ang_vel(3)/Iy;
                   (Ix-Iy)*ang_vel(1)*ang_vel(2)/Iz];
                 
ang_vel_dot2 =    -mu*omega_mag*ang_vel./I;
                 
ang_vel_dot3 =    Gc./I;

angvel_dot = ang_vel_dot1 + ang_vel_dot2 + ang_vel_dot3;

% Final State Derivative Vector
var_dot = [inerital_pos_dot;euler_angle_rates;vel_dot;angvel_dot];

end