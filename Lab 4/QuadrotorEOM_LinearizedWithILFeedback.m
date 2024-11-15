function [var_dot,Lc,Mc,Nc] = QuadrotorEOM_LinearizedWithILFeedback(~, var, g, m, I)

%----------------------------------------------------------------
% Inputs: t = time
%         var = 12x1 column of state deviations from steady hover
%         g = gravity
%         m = quadcopter mass
%         I = inertia matrix
%
% Output: var_dot = 12x1 state vector derivative
%         Lc: X control moment
%         Mc: Y control moment
%         Nc: Z control moment
%----------------------------------------------------------------

% Extract var Components
delta_phi = var(4);
delta_theta = var(5);
delta_u = var(7);
delta_v = var(8);
delta_w = var(9);
delta_p = var(10);
delta_q = var(11);
delta_r = var(12);

% Extract I Components
Ix = I(1);
Iy = I(2);
Iz = I(3);

% InnerLoopFeedBack Call
[Fc,Gc] = InnerLoopFeedback(var);
deltaFc = 0;
Fc = deltaFc;
Lc = Gc(1);
Mc = Gc(2);
Nc = Gc(3);

% Linearized EOMs
delta_xE_dot = delta_u;
delta_yE_dot = delta_v;
delta_zE_dot = delta_w;

delta_phi_dot = delta_p;
delta_theta_dot = delta_q;
delta_psi_dot = delta_r;

delta_u_dot = - g * delta_theta;
delta_v_dot = g * delta_phi;
delta_w_dot = Fc / m;

delta_p_dot = Gc(1) / Ix;
delta_q_dot = Gc(2) / Iy;
delta_r_dot = Gc(3) / Iz;

% Final State Derivative Vector
var_dot = [delta_xE_dot;delta_yE_dot;delta_zE_dot
    delta_phi_dot;delta_theta_dot;delta_psi_dot
    delta_u_dot;delta_v_dot;delta_w_dot
    delta_p_dot;delta_q_dot;delta_r_dot];

end