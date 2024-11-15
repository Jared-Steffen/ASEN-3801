clc
clear 
close all

ttwistor;
wind_inertial = [0;0;0];
tfinal = 200;
TSPAN = [0 tfinal];

%% Problem 2_1

aircraft_state_2_1 = [0;0;-1609.34;0;0;0;21;0;0;0;0;0];
control_input_2_1 = [0;0;0;0];

[TOUT2_1,YOUT2_1] = ode45(@(t,y) AircraftEOM(t,y,control_input_2_1,wind_inertial,aircraft_parameters),TSPAN,aircraft_state_2_1,[]);

for i=1:length(TOUT2_1)
    UOUT2_1(:,i) = control_input_2_1;
end

PlotSimulation(TOUT2_1,YOUT2_1,UOUT2_1,'b')

%% Problem 2_2

aircraft_state_2_2 = [0;0;-1800;0;0.02780;0;20.99;0;0.5837;0;0;0];
control_input_2_2 = [0.1079;0;0;0.3182];

[TOUT2_2,YOUT2_2] = ode45(@(t,y) AircraftEOM(t,y,control_input_2_2,wind_inertial,aircraft_parameters),TSPAN,aircraft_state_2_2,[]);

for i=1:length(TOUT2_2)
    UOUT2_2(:,i) = control_input_2_2;
end

% PlotSimulation(TOUT2_2,YOUT2_2,UOUT2_2,'b')

%% Problem 2_3

aircraft_state_2_3 = [0;0;-1800;deg2rad(15);deg2rad(-12);deg2rad(270);19;3;-2;deg2rad(0.08);deg2rad(-0.2);0];
control_input_2_3 = [deg2rad(5);deg2rad(2);deg2rad(-13);0.3];

[TOUT2_3,YOUT2_3] = ode45(@(t,y) AircraftEOM(t,y,control_input_2_3,wind_inertial,aircraft_parameters),TSPAN,aircraft_state_2_3,[]);

for i=1:length(TOUT2_3)
    UOUT2_3(:,i) = control_input_2_3;
end

% PlotSimulation(TOUT2_3,YOUT2_3,UOUT2_3,'b')

%% Problem 3_1

aircraft_state_3_1 = [0;0;-1800;0;0.02780;0;20.99;0;0.5837;0;0;0];
control_input_3_1 = [0.1079;0;0;0.3182];
d_size_3_1 = 15;
d_time_3_1 = 0.25;

[TOUT3_1,YOUT3_1] = ode45(@(t,y) AircraftEOMDoublet(t,y,control_input_3_1,d_size_3_1,d_time_3_1,wind_inertial,aircraft_parameters),[0 3],aircraft_state_3_1,[]);

UOUT3_1 = zeros(4,length(TOUT3_1));
for i=1:length(TOUT3_1)
    if TOUT3_1(i) > 0 && TOUT3_1(i) <= d_time_3_1
        UOUT3_1(1,i) = control_input_3_1(1) + deg2rad(d_size_3_1);
        UOUT3_1(2:4,i) = control_input_3_1(2:4);
    elseif TOUT3_1(i) > d_time_3_1 && TOUT3_1(i) <= 2*d_time_3_1
        UOUT3_1(1,i) = control_input_3_1(1) - deg2rad(d_size_3_1);
        UOUT3_1(2:4,i) = control_input_3_1(2:4);
    elseif TOUT3_1(i) > 2*d_time_3_1
        UOUT3_1(1,i) = control_input_3_1(1);
        UOUT3_1(2:4,i) = control_input_3_1(2:4);
    end
end

% PlotSimulation(TOUT3_1,YOUT3_1,UOUT3_1,'b')

%% Problem 3_2

[TOUT3_2,YOUT3_2] = ode45(@(t,y) AircraftEOMDoublet(t,y,control_input_3_1,d_size_3_1,d_time_3_1,wind_inertial,aircraft_parameters),[0 100],aircraft_state_3_1,[]);

UOUT3_2 = zeros(4,length(TOUT3_2));
for i=1:length(TOUT3_2)
    if TOUT3_2(i) > 0 && TOUT3_2(i) <= d_time_3_1
        UOUT3_2(1,i) = control_input_3_1(1) + deg2rad(d_size_3_1);
        UOUT3_2(2:4,i) = control_input_3_1(2:4);
    elseif TOUT3_2(i) > d_time_3_1 && TOUT3_2(i) <= 2*d_time_3_1
        UOUT3_2(1,i) = control_input_3_1(1) - deg2rad(d_size_3_1);
        UOUT3_2(2:4,i) = control_input_3_1(2:4);
    elseif TOUT3_2(i) > 2*d_time_3_1
        UOUT3_2(1,i) = control_input_3_1(1);
        UOUT3_2(2:4,i) = control_input_3_1(2:4);
    end
end

% PlotSimulation(TOUT3_2,YOUT3_2,UOUT3_2,'b')

%% Functions

function PlotSimulation(time, aircraft_state_array, control_input_array, col)

control_input_array = [rad2deg(control_input_array(1,:));rad2deg(control_input_array(2,:));rad2deg(control_input_array(3,:));control_input_array(4,:)];

figure(1);
subplot(311);
plot(time, aircraft_state_array(:,1), col); hold on;
title('X Position')
xlabel('Time')
ylabel('Meters')
subplot(312);
plot(time, aircraft_state_array(:,2), col); hold on;
title('Y Position')
xlabel('Time')
ylabel('Meters')
subplot(313);
plot(time, -aircraft_state_array(:,3), col); hold on;
title('Z Position')
xlabel('Time')
ylabel('Meters')

figure(2);
subplot(311);
plot(time, aircraft_state_array(:,4), col); hold on;
title('Roll')
xlabel('Time')
ylabel('Radians')
subplot(312);
plot(time, aircraft_state_array(:,5), col); hold on;
title('Pitch')
xlabel('Time')
ylabel('Radians')
subplot(313);
plot(time, aircraft_state_array(:,6), col); hold on;
title('Yaw')
xlabel('Time')
ylabel('Radians')

figure(3);
subplot(311);
plot(time, aircraft_state_array(:,7), col); hold on;
title('U Value')
xlabel('Time')
ylabel('Velocity (m/s)')
subplot(312);
plot(time, aircraft_state_array(:,8), col); hold on;
title('V Value')
xlabel('Time')
ylabel('Velocity (m/s)')
subplot(313);
plot(time, aircraft_state_array(:,9), col); hold on;
title('W Value')
xlabel('Time')
ylabel('Velocity (m/s)')

figure(4);
subplot(311);
plot(time, aircraft_state_array(:,10), col); hold on;
title('P Value')
xlabel('Time')
ylabel('\omega (radians/s)')
subplot(312);
plot(time, aircraft_state_array(:,11), col); hold on;
title('Q Value')
xlabel('Time')
ylabel('\omega (radians/s)')
subplot(313);
plot(time, aircraft_state_array(:,12), col); hold on;
title('R Value')
xlabel('Time')
ylabel('\omega (radians/s)')

figure(5);
subplot(411);
plot(time,control_input_array(1,:), col); hold on;
title('Elevator Deflection')
xlabel('Time')
ylabel('Degrees')
subplot(412);
plot(time,control_input_array(2,:), col); hold on;
title('Aileron Deflection')
xlabel('Time')
ylabel('Degrees')
subplot(413);
plot(time,control_input_array(3,:), col); hold on;
title('Rudder Deflection')
xlabel('Time')
ylabel('Degrees')
subplot(414);
plot(time,control_input_array(4,:), col); hold on;
title('Throttle')
xlabel('Time')
ylabel('Fraction')

figure(6)
hold on
grid on
plot3(aircraft_state_array(:,1),aircraft_state_array(:,2),-aircraft_state_array(:,3))
plot3(aircraft_state_array(1,1),aircraft_state_array(1,2),-aircraft_state_array(1,3),'og')
plot3(aircraft_state_array(end,1),aircraft_state_array(end,2),-aircraft_state_array(end,3),'or')
zlim([0 2000])
view(3)
xlabel('X distance (meters)')
ylabel('Y distance (meters)')
zlabel('Z distance (meters)')
xlim([0 2500])
zlim([1420 1620])

end

function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

pos_inertial = aircraft_state(1:3,1);
euler_angles = aircraft_state(4:6,1);
vel_body = aircraft_state(7:9,1);
omega_body = aircraft_state(10:12,1);

%%% Kinematics
vel_inertial = TransformFromBodyToInertial(vel_body, euler_angles);
euler_rates = EulerRatesFromOmegaBody(omega_body, euler_angles);


%%% Aerodynamic force and moment
density = stdatmo(-pos_inertial(3,1));

[fa_body, ma_body] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);


%%% Gravity
fg_body = (aircraft_parameters.g)*[-sin(euler_angles(2));sin(euler_angles(1))*cos(euler_angles(2));cos(euler_angles(2))*cos(euler_angles(1))];


%%% Dynamics
vel_body_dot = -cross(omega_body, vel_body) + fg_body+(fa_body)/aircraft_parameters.m;


inertia_matrix = [aircraft_parameters.Ix 0 -aircraft_parameters.Ixz;...
                    0 aircraft_parameters.Iy 0;...
                    -aircraft_parameters.Ixz 0 aircraft_parameters.Iz];

omega_body_dot = inv(inertia_matrix)*(-cross(omega_body, inertia_matrix*omega_body) + ma_body);


%%% State derivative
xdot = [vel_inertial; euler_rates; vel_body_dot; omega_body_dot];

end

function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, doublet_time, wind_inertial, aircraft_parameters)
    
pos_inertial = aircraft_state(1:3,1);
euler_angles = aircraft_state(4:6,1);
vel_body = aircraft_state(7:9,1);
omega_body = aircraft_state(10:12,1);

if time > 0 && time <= doublet_time
    aircraft_surfaces(1) = aircraft_surfaces(1) + doublet_size;
elseif time > doublet_time && time <= 2*doublet_time
    aircraft_surfaces(1) = aircraft_surfaces(1) - doublet_size;
elseif time > 2*doublet_time
    aircraft_surfaces(1) = aircraft_surfaces(1) ;
end


%%% Kinematics
vel_inertial = TransformFromBodyToInertial(vel_body, euler_angles);
euler_rates = EulerRatesFromOmegaBody(omega_body, euler_angles);


%%% Aerodynamic force and moment
density = stdatmo(-pos_inertial(3,1));

[fa_body, ma_body] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);


%%% Gravity
fg_body = (aircraft_parameters.g)*[-sin(euler_angles(2));sin(euler_angles(1))*cos(euler_angles(2));cos(euler_angles(2))*cos(euler_angles(1))];


%%% Dynamics
vel_body_dot = -cross(omega_body, vel_body) + fg_body+(fa_body)/aircraft_parameters.m;


inertia_matrix = [aircraft_parameters.Ix 0 -aircraft_parameters.Ixz;...
                    0 aircraft_parameters.Iy 0;...
                    -aircraft_parameters.Ixz 0 aircraft_parameters.Iz];

omega_body_dot = inv(inertia_matrix)*(-cross(omega_body, inertia_matrix*omega_body) + ma_body);


%%% State derivative
xdot = [vel_inertial; euler_rates; vel_body_dot; omega_body_dot];

end