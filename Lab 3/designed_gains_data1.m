clc
clear

% Expected Result in MATLAB
num = 5.709;
den = [1 3.9938 11.4181];
sys = tf(num,den);
t = [0:0.1:5];
figure();
plot(t,step(sys,t))
ylabel('Position [rad]')
xlabel('Time [s]')
grid on; grid minor
title('Expected Step Response')

% Load Data
data = load("Tested_Gains3");
time = data(:,1); %[ms]
ref_pos = data(:,2); %[rad]
meas_pos = data(:,3); %[rad]

% Adjust/Convert Time Vector
time = time - time(2);
time(1) = 0;
time = time ./ 1000; %[s]

% Plot Data
figure();
plot(time,ref_pos)
hold on
plot(time,meas_pos)
hold off
ylabel('Position [rad]')
xlabel('Time [s]')
grid on; grid minor
title('Designed Control Gains Response')