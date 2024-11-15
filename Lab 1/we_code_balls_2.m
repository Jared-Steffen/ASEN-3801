% Hannah Priddy, Chloe Zentner, Ben Reynolds, Jared Steffen
% ASEN 3801
% Lab 1
% Created: 9/8/23
clear; close all; clc;

m = .050; %kg
g = [0;0;9.81];%m/s^2
d = .02;%m
A = pi*(d/2).^2;
Cd = 0.6;
altitude = linspace(1,1655);
rho1 = stdatmo(1655,0);%kg/m^3
WindSpeeds = 1:100;%m/s

%Pre-allocate
MagDisp = zeros(1,100);
xdisp = zeros(1,100);
min_landing = zeros(1,5);

for R = WindSpeeds
    wind = [R;0;0]; %
    xinit = [0; 0; 0; 0; 20; -20];
    initalvalues = [];
    tspan = linspace(0,30);
    options = odeset('Events',@myEventsFcn);
    [time, positionvec] = ode45(@(t,x) objectEOM(t,x, rho1, Cd, A, m, g, wind), tspan, xinit, options);
    MagDisp(R) = norm(positionvec(end,1:2));
    xdisp(R) = positionvec(end,1) ;
end

% Lab 1 part 2.b
figure(1)
plot3(positionvec(:,1),positionvec(:,2), -positionvec(:,3))
xlabel('North/South')
ylabel('East/West')
zlabel('Up/Down')
legend('Trajectory')
title('2.b Trajectory of Ball')
grid on; grid minor;

% Lab 1 part 2.c
figure(2)
plot(WindSpeeds,MagDisp)
xlabel('Wind Speed[m/s]')
ylabel('Total Meters of Deflection[m]')
title('2.c Wind Speed VS Total Meters of Deflection')
grid on; grid minor;

figure(3)
plot(WindSpeeds,xdisp)
xlabel('Wind Speed[m/s]')
ylabel('X meters of Deflection[m]')
title('2.c Wind Speed VS X Meters of Deflection')
grid on; grid minor;

% Lab 1 part 2.d
j = 1;
for i = 1:2000:8001
    rho2 = stdatmo(i,0);
    for R = WindSpeeds
        wind = [R;0;0]; %
        xinit = [0; 0; 0; 0; 20; -20];
        initalvalues = [];
        tspan = linspace(0,30);
        options = odeset('Events',@myEventsFcn);
        [time, positionvec] = ode45(@(t,x) objectEOM(t,x, rho2, Cd, A, m, g, wind), tspan, xinit, options);
        MagDisp(R) = norm(positionvec(end,1:2));
        xdisp(R) = positionvec(end,1);
    end
    min_landing(j) = min(MagDisp);
    j = j+1;
    figure(4)
    plot(WindSpeeds,MagDisp)
    hold on
end
hold off  
xlabel('Wind Speed[m/s]')
ylabel('Total Meters of Deflection[m]')
title('2.d Wind Speed VS Total Meters of Deflection for Various Altitudes')
legend('0m (Sea Level)','2000m','4000m','6000m','8000m (Mt Everest)','Location','NorthWest')
grid on;grid minor;
% 
figure(5)
plot(1:2000:8001,min_landing)
ylabel('Minimum Landing Distance[m]')
xlabel('Altitude[m]')
title('2.d Minimum Landing Distance vs Various Altitudes')
grid on; grid minor;


% Lab 1 part 2.e
xinit = [0; 0; 0; 0; 20; -20];
max_KE = (1/2)*m*(norm(xinit(4:6)))^2; 
masses = 0.025:0.025:1;

v = sqrt((2*max_KE)./masses);


x_0_2(:,5:6) = [-sqrt((v.^2)/2),sqrt((v.^2)/2)];
y_pos = zeros(0.025/1,1)';

wind = [0 0 0];
t_span = [0 10];

for i = 1:(1/0.025) 
    [time, positionvec] = ode45(@(t,x) objectEOM(t,x, rho1, Cd, A, masses(i), g, wind), tspan, x_0_2(i,:), options);
    Mag(i) = norm(positionvec(end,1:2));
end

plot(masses,Mag)

function xdot = objectEOM(~,x,rho,Cd,A,m,g,wind)
relvelocity = -wind + x(4:6);
airspeed = norm(relvelocity);
Dragscalar = -0.5*Cd*A*rho * airspeed^2;
Drag = Dragscalar*relvelocity/airspeed;
fgrav = m*g;
forces = (Drag+fgrav)/m;
xdot = [x(4:6); forces];

end

function [value,isterminal,direction] = myEventsFcn(~,y)
value = y(3);
isterminal = 1;
direction = 0;
end