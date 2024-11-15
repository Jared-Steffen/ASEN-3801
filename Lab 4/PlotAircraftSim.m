% Question 1
function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

%----------------------------------------------------------------
% Inputs: length n time of nth set of variables
%         12xn aircraft states
%         4xn control inputs (Zc,Lc,Mc,Nc)
%         6x1 figure numbers to plot over
%         string col
% Outputs: 6 figures
%                 4 figs w/ 3 subplot (inertial pos, Euler angle, interial
%                        velocity in body, angular velocity)
%                 1 fig w/ 4 subplot (each control input variable)
%                 1 fig w/ 3D path (pos height up, start green, finish red)
% Methodology: plot full aircraft states and control inputs
%----------------------------------------------------------------

% Inertial Position
figure(fig(1)); 
subplot(311); % x direction
plot(time, aircraft_state_array(:,1), col); hold on;
title('Inertial Position')
xlabel('time [s]'); ylabel('x [m]');
subplot(312); % y direction
plot(time, aircraft_state_array(:,2), col); hold on;
xlabel('time [s]'); ylabel('y [m]')
subplot(313); % z direction
plot(time, aircraft_state_array(:,3), col); hold on;
xlabel('time [s]'); ylabel('z [m]');


% Euler Angles
figure(fig(2));
subplot(311);
plot(time, aircraft_state_array(:,4), col); hold on;
title('Euler Angles')
xlabel('time [s]'); ylabel('\phi [rad]');
subplot(312);
plot(time, aircraft_state_array(:,5), col); hold on;
xlabel('time [s]'); ylabel('\theta [rad]');
subplot(313);
plot(time, aircraft_state_array(:,6), col); hold on;
xlabel('time [s]'); ylabel('\psi [rad]');


% Inertial Velocity in Body Frame
figure(fig(3));
subplot(311);
plot(time, aircraft_state_array(:,7), col); hold on;
title('Inertial Velocity in Body Frame')
xlabel('time [s]'); ylabel('u^{E} [m/s]');
subplot(312);
plot(time, aircraft_state_array(:,8), col); hold on;
xlabel('time [s]'); ylabel('v^{E} [m/s]');
subplot(313);
plot(time, aircraft_state_array(:,9), col); hold on;
xlabel('time [s]'); ylabel('w^{E} [m/s]');


% Angular Velocity
figure(fig(4));
subplot(311);
plot(time, aircraft_state_array(:,10), col); hold on;
title('Angular Velocity')
xlabel('time [s]'); ylabel('p [rad/s]');
subplot(312);
plot(time, aircraft_state_array(:,11), col); hold on;
xlabel('time [s]'); ylabel('q [rad/s]');
subplot(313);
plot(time, aircraft_state_array(:,12), col); hold on;
xlabel('time [s]'); ylabel('r [rad/s]');

% Control Input Variables
Zc = control_input_array(1,:);
Lc = control_input_array(2,:);
Mc = control_input_array(3,:);
Nc = control_input_array(4,:);
figure(fig(5));
subplot(411);
plot(time, Zc, col); hold on;
title('Control Input Variables')
xlabel('time [s]'); ylabel('Z_{c} [N]');
subplot(412);
plot(time, Lc, col); hold on;
xlabel('time [s]'); ylabel('L_{c} [Nm]');
% ylim([-.001 0.001])
subplot(413);
plot(time, Mc, col); hold on;
xlabel('time [s]'); ylabel('M_{c} [Nm]');
% ylim([-.001 0.001])
subplot(414);
plot(time, Nc, col); hold on;
xlabel('time [s]'); ylabel('N_{c} [Nm]');
% ylim([-.001 0.001])


% 3D Path of Aircraft
len = length(time);
figure(fig(6))
plot3(aircraft_state_array(:,1),aircraft_state_array(:,2),aircraft_state_array(:,3),col)
hold on
grid on; grid minor;
title('3D Path of Aircraft')
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
plot3(aircraft_state_array(1,1),aircraft_state_array(1,2),aircraft_state_array(1,3),'.g','MarkerSize',12); hold on;
plot3(aircraft_state_array(len,1),aircraft_state_array(len,2),aircraft_state_array(len,3),'.r','MarkerSize',12); hold on;

end