clc
clear
close all

%% Constants

% Quadrotor Constants
m = 0.068; %[kg]
d = 0.060; %[m]
km = 0.0024; %[Nm/N]
Ix = 5.8 * 10^-5; %[kgm^2]
Iy = 7.2 * 10^-5; %[kgm^2]
Iz = 1.0 * 10^-4; %[kgm^2]
nu = 1.0 * 10^-3; %[N/(m/s)^2]
mu = 2.0 * 10^-6; %[Nm/(rad/s)^2]
g = 9.81; %[m/s^2]

% Inertia Matrix
I = [Ix;Iy;Iz];

%% 1.2/1.3

% Initial Conditions
var_a = [0;0;0;0;0;0;0;0;0;0;0;0];

% Time
tspan = [0 10]; %[s]

% Motor Forces/Figure Vectors
W = m*g; %[N]
Zc_a = -W;
motor_forces_a = [-Zc_a/4; -Zc_a/4; -Zc_a/4; -Zc_a/4];
controls_a = [Zc_a,0,0,0];
Gc_a = [0;0;0];
fig_a = 1:6;

% ode45 Call
[ta,var_dot_a] = ode45(@(tspan,var_a) QuadrotorEOM(tspan,var_a,g,m,I,d,km,nu,mu,motor_forces_a),tspan,var_a);

controls_a = [Zc_a;0;0;0] .* ones(length(ta),1)';

% PlotAircraftSim Call
% PlotAircraftSim(ta,var_dot_a,controls_a,fig_a,'-b')

%% 1.4a

% Calculate Drag
Va = 5; %[m/s]
drag = nu*Va^2;

% Calculate Phi
phi = deg2rad(atand(-m*g/drag) + 90);

% Calculate Zc
Zc_b = -m*g/cos(phi);

% Calculate v_E and w_E
theta = 0;
psi = 0;
inert_vel = [0;5;0];

cPH = cos(phi);
cT = cos(theta);
cPS = cos(psi);
sPH = sin(phi);
sT = sin(theta);
sPS = sin(psi);

Rot321 = [cT*cPS,sPH*sT*cPS-cPH*sPS,cPH*sT*cPS+sPH*sPS;
             cT*sPS,sPH*sT*sPS+cPH*cPS,cPH*sT*sPS-sPH*cPS;
             -sT,sPH*cT,cPH*cT];

inert_vel_bb = Rot321' * inert_vel;

% Initial Conditions
var_b = [0;0;0;phi;0;0;inert_vel_bb(1);inert_vel_bb(2);inert_vel_bb(3);0;0;0];

% Motor Forces/Figure Vectors
motor_forces_b = [-Zc_b/4; -Zc_b/4; -Zc_b/4; -Zc_b/4];
fig_b = 7:12;

% ode45 Call
[tb,var_dot_b] = ode45(@(tspan,var_b) QuadrotorEOM(tspan,var_b,g,m,I,d,km,nu,mu,motor_forces_b),tspan,var_b);

% Controls Vector
controls_b = [Zc_b;0;0;0] .* ones(length(tb),1)';

% PlotAircraftSim Call
% PlotAircraftSim(tb,var_dot_b,controls_b,fig_b,'-b')

%% 1.4b

% Calculate Theta
theta = -deg2rad(atand(-m*g/drag) + 90);

% Calculate Zc
Zc_c = -m*g/cos(theta);

% Calculate u_E and w_E
phi = 0;
psi = deg2rad(90);

cPH = cos(phi);
cT = cos(theta);
cPS = cos(psi);
sPH = sin(phi);
sT = sin(theta);
sPS = sin(psi);

Rot321 = [cT*cPS,sPH*sT*cPS-cPH*sPS,cPH*sT*cPS+sPH*sPS;
             cT*sPS,sPH*sT*sPS+cPH*cPS,cPH*sT*sPS-sPH*cPS;
             -sT,sPH*cT,cPH*cT];

inert_vel_bc = Rot321' * inert_vel;


% Initial Conditions
var_c = [0;0;0;0;theta;psi;inert_vel_bc(1);inert_vel_bc(2);inert_vel_bc(3);0;0;0];

% Motor Forces/Control/Figure Vectors
motor_forces_c = [-Zc_c/4; -Zc_c/4; -Zc_c/4; -Zc_c/4];
fig_c = 13:18;

% ode45 Call
[tc,var_dot_c] = ode45(@(tspan,var_c) QuadrotorEOM(tspan,var_c,g,m,I,d,km,nu,mu,motor_forces_c),tspan,var_c);

% Controls Vector
controls_c = [Zc_c;0;0;0].* ones(length(tc),1)';

% PlotAircraftSim Call
% PlotAircraftSim(tc,var_dot_c,controls_c,fig_c,'-b')

%% 1.5

% Import Hardware Data
load("RSdata_nocontrol.mat")

% Figure Vector
fig = 19:24;


% PlotAircraftSim Call
% PlotAircraftSim(rt_estim.time,rt_estim.signals.values,sqrt(abs(rt_motor.signals.values)*13840.4)',fig,'-b')
% Note: Not sure how to get Zc, Lc, Mc, Nc from data, so those graphs can be ignored
%% 2.1/2.2

% Initial Conditions
var_d = [0;0;0;deg2rad(5);0;0;0;0;0;0;0;0];
var_e = [0;0;0;0;deg2rad(5);0;0;0;0;0;0;0];
var_f = [0;0;0;0;0;deg2rad(5);0;0;0;0;0;0];
var_g = [0;0;0;0;0;0;0;0;0;0.1;0;0];
var_h = [0;0;0;0;0;0;0;0;0;0;0.1;0];
var_i = [0;0;0;0;0;0;0;0;0;0;0;0.1];

% Motor Forces and Moments
deltaFc = 0;
deltaGc = [0;0;0];

% Nonlinear ode45 Call
[td,var_dot_d] = ode45(@(tspan,var_d) QuadrotorEOM(tspan,var_d,g,m,I,d,km,nu,mu,motor_forces_a),tspan,var_d);
[te,var_dot_e] = ode45(@(tspan,var_e) QuadrotorEOM(tspan,var_e,g,m,I,d,km,nu,mu,motor_forces_a),tspan,var_e);
[tf,var_dot_f] = ode45(@(tspan,var_f) QuadrotorEOM(tspan,var_f,g,m,I,d,km,nu,mu,motor_forces_a),tspan,var_f);
[tg,var_dot_g] = ode45(@(tspan,var_g) QuadrotorEOM(tspan,var_g,g,m,I,d,km,nu,mu,motor_forces_a),tspan,var_g);
[th,var_dot_h] = ode45(@(tspan,var_h) QuadrotorEOM(tspan,var_h,g,m,I,d,km,nu,mu,motor_forces_a),tspan,var_h);
[ti,var_dot_i] = ode45(@(tspan,var_i) QuadrotorEOM(tspan,var_i,g,m,I,d,km,nu,mu,motor_forces_a),tspan,var_i);

% Linear ode45 Call
[tdL,var_dot_dL] = ode45(@(tspan,var_d) QuadrotorEOM_Linearized(tspan,var_d,g,m,I,deltaFc,deltaGc),tspan,var_d);
[teL,var_dot_eL] = ode45(@(tspan,var_e) QuadrotorEOM_Linearized(tspan,var_e,g,m,I,deltaFc,deltaGc),tspan,var_e);
[tfL,var_dot_fL] = ode45(@(tspan,var_f) QuadrotorEOM_Linearized(tspan,var_f,g,m,I,deltaFc,deltaGc),tspan,var_f);
[tgL,var_dot_gL] = ode45(@(tspan,var_g) QuadrotorEOM_Linearized(tspan,var_g,g,m,I,deltaFc,deltaGc),tspan,var_g);
[thL,var_dot_hL] = ode45(@(tspan,var_h) QuadrotorEOM_Linearized(tspan,var_h,g,m,I,deltaFc,deltaGc),tspan,var_h);
[tiL,var_dot_iL] = ode45(@(tspan,var_i) QuadrotorEOM_Linearized(tspan,var_i,g,m,I,deltaFc,deltaGc),tspan,var_i);

% Figure Vector
fig_d = 25:30;
fig_e = 31:36;
fig_f = 37:42;
fig_g = 43:48;
fig_h = 49:54;
fig_i = 55:60;

% Control Vectors
controls_d = [Zc_a;0;0;0] .* ones(length(td),1)';
controls_e = [Zc_a;0;0;0] .* ones(length(te),1)';
controls_f = [Zc_a;0;0;0] .* ones(length(tf),1)';
controls_g = [Zc_a;0;0;0] .* ones(length(tg),1)';
controls_h = [Zc_a;0;0;0] .* ones(length(th),1)';
controls_i = [Zc_a;0;0;0] .* ones(length(ti),1)';
controls_dL = [Zc_a;0;0;0] .* ones(length(tdL),1)';
controls_eL = [Zc_a;0;0;0] .* ones(length(teL),1)';
controls_fL = [Zc_a;0;0;0] .* ones(length(tfL),1)';
controls_gL = [Zc_a;0;0;0] .* ones(length(tgL),1)';
controls_hL = [Zc_a;0;0;0] .* ones(length(thL),1)';
controls_iL = [Zc_a;0;0;0] .* ones(length(tiL),1)';

% PlotAircraftSim Calls
% Nonlinear
% PlotAircraftSim(td,var_dot_d,controls_d,fig_d,'-b')
% PlotAircraftSim(te,var_dot_e,controls_e,fig_e,'-b')
% PlotAircraftSim(tf,var_dot_f,controls_f,fig_f,'-b')
% PlotAircraftSim(tg,var_dot_g,controls_g,fig_g,'-b')
% PlotAircraftSim(th,var_dot_h,controls_h,fig_h,'-b')
% PlotAircraftSim(ti,var_dot_i,controls_i,fig_i,'-b')

% Linear
% PlotAircraftSim(tdL,var_dot_dL,controls_dL,fig_d,'--r')
% PlotAircraftSim(teL,var_dot_eL,controls_eL,fig_e,'--r')
% PlotAircraftSim(tfL,var_dot_fL,controls_fL,fig_f,'--r')
% PlotAircraftSim(tgL,var_dot_gL,controls_gL,fig_g,'--r')
% PlotAircraftSim(thL,var_dot_hL,controls_hL,fig_h,'--r')
% PlotAircraftSim(tiL,var_dot_iL,controls_iL,fig_i,'--r')

%% 2.5

% RotationDerivativeFeedback Calls
[Fc_g,Gc_g] = RotationDerivativeFeedback(var_g,m,g);
[Fc_h,Gc_h] = RotationDerivativeFeedback(var_h,m,g);
[Fc_i,Gc_i] = RotationDerivativeFeedback(var_i,m,g);

% ode45 Calls
[tgRF,var_dot_gRF] = ode45(@(tspan,var_g) QuadrotorEOMwithRateFeedback(tspan,var_g,g,m,I,nu,mu),tspan,var_g);
[thRF,var_dot_hRF] = ode45(@(tspan,var_h) QuadrotorEOMwithRateFeedback(tspan,var_h,g,m,I,nu,mu),tspan,var_h);
[tiRF,var_dot_iRF] = ode45(@(tspan,var_i) QuadrotorEOMwithRateFeedback(tspan,var_i,g,m,I,nu,mu),tspan,var_i);

% Control Moment and Force Vectors
for k = 1:numel(tgRF)
    [~,Lc_g(:,k),Mc_g(:,k),Nc_g(:,k)] = QuadrotorEOMwithRateFeedback(tgRF(k),var_dot_gRF(k,:), g, m, I, nu, mu);
end

for k = 1:numel(thRF)
    [~,Lc_h(:,k),Mc_h(:,k),Nc_h(:,k)] = QuadrotorEOMwithRateFeedback(thRF(k),var_dot_hRF(k,:), g, m, I, nu, mu);
end

for k = 1:numel(tiRF)
    [~,Lc_i(:,k),Mc_i(:,k),Nc_i(:,k)] = QuadrotorEOMwithRateFeedback(tiRF(k),var_dot_iRF(k,:), g, m, I, nu, mu);
end

Fc_g = (Fc_g * ones(length(Lc_g),1))';
Fc_h = (Fc_h * ones(length(Lc_h),1))';
Fc_i = (Fc_i * ones(length(Lc_i),1))';

Gc_g = [Lc_g;Mc_g;Nc_g];
Gc_h = [Lc_h;Mc_h;Nc_h];
Gc_i = [Lc_i;Mc_i;Nc_i];

% Control/Figure Vectors 
controls_gRF = [Fc_g;Lc_g;Mc_g;Nc_g];
controls_hRF = [Fc_h;Lc_h;Mc_h;Nc_h];
controls_iRF = [Fc_i;Lc_i;Mc_i;Nc_i];
fig_gRF = 61:66;
fig_hRF = 67:72;
fig_iRF = 73:78;

% PlotAircraftSim Calls
% Controlled
% PlotAircraftSim(tgRF,var_dot_gRF,controls_gRF,fig_gRF,'-b')
% PlotAircraftSim(thRF,var_dot_hRF,controls_hRF,fig_hRF,'-b')
% PlotAircraftSim(tiRF,var_dot_iRF,controls_iRF,fig_iRF,'-b')

% % Uncontrolled
% PlotAircraftSim(tg,var_dot_g,controls_g,fig_gRF,'--r')
% PlotAircraftSim(th,var_dot_h,controls_h,fig_hRF,'--r')
% PlotAircraftSim(ti,var_dot_i,controls_i,fig_iRF,'--r')

% ComputeMotorForces Call for Controlled and Uncontrolled Systems
% Controlled
motor_forces_gRF = ComputeMotorForces(Fc_g,Gc_g,km,d);
motor_forces_hRF = ComputeMotorForces(Fc_h,Gc_h,km,d);
motor_forces_iRF = ComputeMotorForces(Fc_i,Gc_i,km,d);

% Uncontrolled
motor_forces_ghi = ComputeMotorForces(Zc_a,Gc_a,km,d);

motor_forces_ghi = motor_forces_ghi .* ones(length(motor_forces_ghi),length(tg));

% Plot Motor Forces for Controlled and Uncontrolled Systems
% figure(79);
% subplot(221)
% plot(tgRF,motor_forces_gRF(1,:),'-b','LineWidth',1.5); hold on;
% plot(thRF,motor_forces_hRF(1,:),'-m','LineWidth',1.2);
% plot(tiRF,motor_forces_iRF(1,:),'-k');
% plot(tg,motor_forces_ghi(1,:),'--r');
% title('f1 Motor Force')
% xlabel('Time [s]')
% ylabel('Force [N]')
% xlim([-0.1 1])
% subplot(222)
% plot(tgRF,motor_forces_gRF(2,:),'-b','LineWidth',1.5); hold on;
% plot(thRF,motor_forces_hRF(2,:),'--m','LineWidth',1.2);
% plot(tiRF,motor_forces_iRF(2,:),'-k');
% plot(tg,motor_forces_ghi(2,:),'--r');
% title('f2 Motor Force')
% xlabel('Time [s]')
% ylabel('Force [N]')
% xlim([-0.1 1])
% subplot(223)
% plot(tgRF,motor_forces_gRF(3,:),'-b','LineWidth',1.5); hold on;
% plot(thRF,motor_forces_hRF(3,:),'--m','LineWidth',1.2);
% plot(tiRF,motor_forces_iRF(3,:),'-k');
% plot(tg,motor_forces_ghi(3,:),'--r')
% title('f3 Motor Force')
% xlabel('Time [s]')
% ylabel('Force [N]')
% xlim([-0.1 1])
% subplot(224)
% plot(tgRF,motor_forces_gRF(4,:),'-b','LineWidth',1.5); hold on;
% plot(thRF,motor_forces_hRF(4,:),'--m','LineWidth',1.2);
% plot(tiRF,motor_forces_iRF(4,:),'-k');
% plot(tg,motor_forces_ghi(4,:),'--r')
% title('f4 Motor Force')
% xlabel('Time [s]')
% ylabel('Force [N]')
% xlim([-0.1 1])
% Lgnd = legend('Controlled Case d','Controlled Case e','Controlled Case f','Uncontrolled','Location','bestoutside');
% Lgnd.Position(1) = -0.06;
% Lgnd.Position(2) = 0.45;


%% 3.1
% Gains were calculated by hand for 3.1

%% 3.3

% ode45 Calls
[tdILF,var_dot_dILF] = ode45(@(tspan,var_d) QuadrotorEOM_LinearizedWithILFeedback(tspan,var_d,g,m,I),tspan,var_d);
[teILF,var_dot_eILF] = ode45(@(tspan,var_e) QuadrotorEOM_LinearizedWithILFeedback(tspan,var_e,g,m,I),tspan,var_e);
[tgILF,var_dot_gILF] = ode45(@(tspan,var_g) QuadrotorEOM_LinearizedWithILFeedback(tspan,var_g,g,m,I),tspan,var_g);
[thILF,var_dot_hILF] = ode45(@(tspan,var_h) QuadrotorEOM_LinearizedWithILFeedback(tspan,var_h,g,m,I),tspan,var_h);

% Control Moment Vectors
for k = 1:numel(tdILF)
    [~,Lc_dILF(:,k),Mc_dILF(:,k),Nc_dILF(:,k)] = QuadrotorEOM_LinearizedWithILFeedback(tdILF(k),var_dot_dILF(k,:), g, m, I);
end

for k = 1:numel(teILF)
    [~,Lc_eILF(:,k),Mc_eILF(:,k),Nc_eILF(:,k)] = QuadrotorEOM_LinearizedWithILFeedback(teILF(k),var_dot_eILF(k,:), g, m, I);
end

for k = 1:numel(tgILF)
    [~,Lc_gILF(:,k),Mc_gILF(:,k),Nc_gILF(:,k)] = QuadrotorEOM_LinearizedWithILFeedback(tgILF(k),var_dot_gILF(k,:), g, m, I);
end

for k = 1:numel(thILF)
    [~,Lc_hILF(:,k),Mc_hILF(:,k),Nc_hILF(:,k)] = QuadrotorEOM_LinearizedWithILFeedback(thILF(k),var_dot_hILF(k,:), g, m, I);
end

% InnerLoopFeedback Calls
[Fc_dILF,Gc_dILF] = InnerLoopFeedback(var_d);
[Fc_eILF,Gc_eILF] = InnerLoopFeedback(var_e);
[Fc_gILF,Gc_gILF] = InnerLoopFeedback(var_g);
[Fc_hILF,Gc_hILF] = InnerLoopFeedback(var_h);

% Control Force Vectors
Fc_dILF = (0 * ones(length(Lc_dILF),1))';
Fc_eILF = (0 * ones(length(Lc_eILF),1))';
Fc_gILF = (0 * ones(length(Lc_gILF),1))';
Fc_hILF = (0 * ones(length(Lc_hILF),1))';

% Control/Figure Vectors
controls_dILF = [Fc_dILF;Lc_dILF;Mc_dILF;Nc_dILF];
controls_eILF = [Fc_eILF;Lc_eILF;Mc_eILF;Nc_eILF];
controls_gILF = [Fc_gILF;Lc_gILF;Mc_gILF;Nc_gILF];
controls_hILF = [Fc_hILF;Lc_hILF;Mc_hILF;Nc_hILF];
fig_dILF = 80:85;
fig_eILF = 86:91;
fig_gILF = 92:97;
fig_hILF = 98:103;

% PlotAircraftSim Calls
% PlotAircraftSim(tdILF,var_dot_dILF,controls_dILF,fig_dILF,'-r')
% PlotAircraftSim(teILF,var_dot_eILF,controls_eILF,fig_eILF,'-r')
% PlotAircraftSim(tgILF,var_dot_gILF,controls_gILF,fig_gILF,'-r')
% PlotAircraftSim(thILF,var_dot_hILF,controls_hILF,fig_hILF,'-r')

%% 3.4

% ode45 Calls
[tdILF2,var_dot_dILF2] = ode45(@(tspan,var_d) QuadrotorEOMWithILFeedback(tspan,var_d,g,m,I,nu,mu),tspan,var_d);
[teILF2,var_dot_eILF2] = ode45(@(tspan,var_e) QuadrotorEOMWithILFeedback(tspan,var_e,g,m,I,nu,mu),tspan,var_e);
[tgILF2,var_dot_gILF2] = ode45(@(tspan,var_g) QuadrotorEOMWithILFeedback(tspan,var_g,g,m,I,nu,mu),tspan,var_g);
[thILF2,var_dot_hILF2] = ode45(@(tspan,var_h) QuadrotorEOMWithILFeedback(tspan,var_h,g,m,I,nu,mu),tspan,var_h);


% Control Moment Vectors
for k = 1:numel(tdILF2)
    [~,Lc_dILF2(:,k),Mc_dILF2(:,k),Nc_dILF2(:,k)] = QuadrotorEOMWithILFeedback(tdILF2(k),var_dot_dILF2(k,:), g, m, I, nu, mu);
end

for k = 1:numel(teILF2)
    [~,Lc_eILF2(:,k),Mc_eILF2(:,k),Nc_eILF2(:,k)] = QuadrotorEOMWithILFeedback(teILF2(k),var_dot_eILF2(k,:), g, m, I, nu, mu);
end

for k = 1:numel(tgILF2)
    [~,Lc_gILF2(:,k),Mc_gILF2(:,k),Nc_gILF2(:,k)] = QuadrotorEOMWithILFeedback(tgILF2(k),var_dot_gILF2(k,:), g, m, I, nu, mu);
end

for k = 1:numel(thILF2)
    [~,Lc_hILF2(:,k),Mc_hILF2(:,k),Nc_hILF2(:,k)] = QuadrotorEOMWithILFeedback(thILF2(k),var_dot_hILF2(k,:), g, m, I, nu, mu);
end

% InnerLoopFeedback Calls
[Fc_dILF2,Gc_dILF2] = InnerLoopFeedback(var_d);
[Fc_eILF2,Gc_eILF2] = InnerLoopFeedback(var_d);
[Fc_gILF2,Gc_gILF2] = InnerLoopFeedback(var_d);
[Fc_hILF2,Gc_hILF2] = InnerLoopFeedback(var_d);

% Control Force Vectors
Fc_dILF2 = (Fc_dILF2 * ones(length(Lc_dILF2),1))';
Fc_eILF2 = (Fc_eILF2 * ones(length(Lc_eILF2),1))';
Fc_gILF2 = (Fc_gILF2 * ones(length(Lc_gILF2),1))';
Fc_hILF2 = (Fc_hILF2 * ones(length(Lc_hILF2),1))';


% Control Vectors
controls_dILF2 = [Fc_dILF2;Lc_dILF2;Mc_dILF2;Nc_dILF2];
controls_eILF2 = [Fc_eILF2;Lc_eILF2;Mc_eILF2;Nc_eILF2];
controls_gILF2 = [Fc_gILF2;Lc_gILF2;Mc_gILF2;Nc_gILF2];
controls_hILF2 = [Fc_hILF2;Lc_hILF2;Mc_hILF2;Nc_hILF2];

% PlotAircraftSim Calls
% PlotAircraftSim(tdILF2,var_dot_dILF2,controls_dILF2,fig_dILF,'-b')
% PlotAircraftSim(teILF2,var_dot_eILF2,controls_eILF2,fig_eILF,'-b')
% PlotAircraftSim(tgILF2,var_dot_gILF2,controls_gILF2,fig_gILF,'-b')
% PlotAircraftSim(thILF2,var_dot_hILF2,controls_hILF2,fig_hILF,'-b')

%% 3.5

% Lateral
% Gains
k1_lat = 0.001276;
k2_lat = 0.00232;
k3_lat = (0:0.05:10)*Ix;

% Find k3_lat for Tau = 1.25s
eigA_lat = zeros(length(k3_lat),3);
j = 1;
for i = 1:length(k3_lat)
    A = [0 g 0;
         0 0 1;
         -k3_lat(i)/Ix -k2_lat/Ix -k1_lat/Ix];
    eigA_lat(j,:) = eig(A);
    figure(104);
    plot(real(eigA_lat),imag(eigA_lat),'.k')
    hold on
    j = j + 1;
end
plot(real(eigA_lat(39,:)),imag(eigA_lat(39,:)),'.g')
xline(-0.8,'--r')
xlabel('Real')
ylabel('Imaginary')
title('Lateral Root Locus Plot')
hold off

k3_lat = k3_lat(39);

% % Longitudinal
% % Gains
% k1_lon = 0.001584;
% k2_lon = 0.00288;
% k3_lon = (0:0.05:10) * -Iy;
% 
% % Find k3_lon for Tau = 1.25s
% eigA_lon = zeros(length(k3_lon),3);
% j = 1;
% for i = 1:length(k3_lon)
%     A = [0 -g 0;
%          0 0 1
%          -k3_lon(i)/Iy -k2_lon/Iy -k1_lon/Iy];
%     eigA_lon(j,:)= eig(A);
%     figure(105);
%     plot(real(eigA_lon), imag(eigA_lon), '.k');
%     hold on
%     j = j + 1;
% end
% plot(real(eigA_lon(39,:)), imag(eigA_lon(39,:)), '.g');
% xline(-.8,'--r');
% xlabel('Real')
% ylabel('Imaginary')
% title('Longitudinal Root Locus Plot')
% hold off
% 
% k3_lon = k3_lon(39);

%% 3.7

% Lateral
% Initial State Vector
var_j = [0;0;0;0;0;0;0;0;0;0;0;0];

% ode45 Call
[tj,var_dot_j] = ode45(@(tspan,var_j) QuadrotorEOMTranslation(tspan,var_j,g,m,I,nu,mu,'Lateral'),tspan,var_j);

% Control Forces/Moments
for k = 1:numel(tj)
    [~,Lc_j(:,k),Mc_j(:,k),Nc_j(:,k)] = QuadrotorEOMTranslation(tj(k),var_dot_j(k,:), g, m, I, nu, mu, 'Lateral');
end

[Fc_j,Gc_j] = VelocityReferenceFeedbackLat(tspan,var_j);
Fc_j = (Fc_j * ones(length(Lc_j),1))';

% Control/Figures Vector
controls_j = [Fc_j;Lc_j;Mc_j;Nc_j];
fig_j = 106:111;

% PlotAircraftSim Call
PlotAircraftSim(tj,var_dot_j,controls_j,fig_j,'-b')

%------------------------------------------------------------------------------------
% Longitudinal
% ode45 Call 
[tk,var_dot_k] = ode45(@(tspan,var_j) QuadrotorEOMTranslation(tspan,var_j,g,m,I,nu,mu,'Longitudinal'),tspan,var_j);

% Control Forces/Moments
for k = 1:numel(tj)
    [~,Lc_k(:,k),Mc_k(:,k),Nc_k(:,k)] = QuadrotorEOMTranslation(tj(k),var_dot_j(k,:), g, m, I, nu, mu, 'Longitudinal');
end

[Fc_k,Gc_k] = VelocityReferenceFeedbackLon(tspan,var_j);
Fc_k = (Fc_k * ones(length(Lc_j),1))';

% Control/Figure Vectors
controls_k = [Fc_k;Lc_k;Mc_k;Nc_k];
fig_k = 112:117;

% PlotAircraftSim Call
% PlotAircraftSim(tk,var_dot_k,controls_k,fig_k,'-b')

%% 3.8
% Tested Gains
% Load Data
gains_data = load('RSdata_10_26.mat');

% Figure Vector
fig_l = 118:123;

% PlotAircraftSim Call
% PlotAircraftSim(gains_data.rt_estim.time,gains_data.rt_estim.signals.values,sqrt(abs(gains_data.rt_motor.signals.values)*13840.4)',fig_l,'-b')
% Note: Not sure how to get Zc, Lc, Mc, Nc from data, so those graphs can be ignored