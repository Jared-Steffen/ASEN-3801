clc
clear
close all

%% Constants

% Quadrotor Constants
m = 10; %[kg]
d = 0.480; %[m]
km = 0.0024; %[Nm/N]
Ix = 0.17215;
Iy = 0.19460;
Iz = 0.20743;
nu = 5.0 * 10^-3; %[N/(m/s)^2]
mu = 2.0 * 10^-6; %[Nm/(rad/s)^2]
g = 9.81; %[m/s^2]

% Inertia Matrix
I = [Ix;Iy;Iz];

tspan = [0 30];
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
% 
% % Lateral
% % Gains
% k1_lat = 22*Ix;
% k2_lat = 40*Ix;
% k3_lat = (0:0.05:10)*Ix;
% 
% % Find k3_lat for Tau = 1.25s
% eigA_lat = zeros(length(k3_lat),3);
% j = 1;
% for i = 1:length(k3_lat)
%     A = [0 g 0;
%          0 0 1;
%          -k3_lat(i)/Ix -k2_lat/Ix -k1_lat/Ix];
%     eigA_lat(j,:) = eig(A);
%     figure(104);
%     plot(real(eigA_lat),imag(eigA_lat),'.k')
%     hold on
%     j = j + 1;
% end
% plot(real(eigA_lat(39,:)),imag(eigA_lat(39,:)),'.g')
% xline(-0.8,'--r')
% xlabel('Real')
% ylabel('Imaginary')
% title('Lateral Root Locus Plot')
% hold off
% 
% k3_lat = k3_lat(39);