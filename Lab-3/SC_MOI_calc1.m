clc; clear

% Load and Break Up Data
data = load("2023_10_13_002_SC_INERTIA");
time = data(:,1); %[ms]
angular_vel = data(:,3); %[rpm]

% Convert Data
time = time ./ 1000; %[s]
angular_vel = angular_vel ./ 60;
angular_vel = 2 * pi .* angular_vel; %[rad/s]

% Remove Data Before 1 second and Post 6 Seconds
time = time(101:601);
angular_vel = angular_vel(101:601);

% Best Fit Line
p = polyfit(time,angular_vel,1);

% Angular Acceleration (slope)
angular_accel = abs(p(1)) %[rad/s^2]

% Torque 
torque = 10 %[mNm]

SC_MOI =  torque/angular_accel