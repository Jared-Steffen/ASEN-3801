%% ASEN 3801 Lab02 Main Script
% Names: Jackson Clark, Robert Kaplan, Eric Meyer, Jared Steffen
% Date Modified: 09/24/2023
% Description: Main script utilizing LoadASPENData, RotationMatrix321/313,
% EulerAngles321 and Convert ASPENData for purposes of responding to Lab02
% prompts

%% House Keeping

% Workspace organization

clc; clear; close all;

%% Assignment General

% Importing data

filename = "3801_Sec2_Test3.csv";

[t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename); % Load in file time, position, and attitude data of each vehicle

%% Assignment Part A

% Visualizing each aircraft in a 3D space

% Plot the inertial positions of each craft relative to time in an X-Y-Z space
figure();
plot3(av_pos_inert(1, :), av_pos_inert(2, :), av_pos_inert(3, :), 'g--', tar_pos_inert(1, :), tar_pos_inert(2, :), tar_pos_inert(3, :), 'b-')
set(gca, 'ZDir','reverse')
view(3)
title("Assignment Part A: Inertial Positions of Vehicle & Target Over Time")
xlabel("X Inertial Position (m)")
ylabel("Y Inertial Position (m)")
zlabel("Z Inertial Position (m)")
legend("Vehicle", "Target")
grid on;
axis equal;

%% Assignment Part B

% Visualizing the isolated position components as well as the 3-2-1 inertial->body rotation angles of each aircraft over time

% Plot each inertial position component (X-Y-Z) individually with respect to time
% for each aircraft
figure();
subplot(3,1,1)
plot(t_vec(:), av_pos_inert(1, :), 'g--', t_vec(:), tar_pos_inert(1, :), 'b-')
title("Assignment Part B: X Component Positions of Vehicle & Target Over Time")
xlabel("Time (s)")
ylabel("X Inertial Position (m)")
legend("Vehicle", "Target")
subplot(3,1,2)
plot(t_vec(:), av_pos_inert(2, :), 'g--', t_vec(:), tar_pos_inert(2, :), 'b-')
title("Assignment Part B: Y Component Positions of Vehicle & Target Over Time")
xlabel("Time (s)")
ylabel("Y Inertial Position (m)")
legend("Vehicle", "Target")
subplot(3,1,3)
plot(t_vec(:), av_pos_inert(3, :), 'g--', t_vec(:), tar_pos_inert(3, :), 'b-')
title("Assignment Part B: Z Component Positions of Vehicle & Target Over Time")
xlabel("Time (s)")
ylabel("Z Inertial Position (m)")
legend("Vehicle", "Target")

% Calculate the rotation angles of each aircraft in the body frame using the imported euler
% angles and an inertial -> body frame 3-2-1 rotation matrix

av_att = av_att .* 180/pi; % Conversion to Degrees
tar_att = tar_att .* 180/pi; % Conversion to Degrees

% EAV321 is Euler Angle of Vehicle Rotated by 3-2-1 Euler Angles (Inertial -> Body)

EAV321 = zeros(3, length(av_att(1, :))); % Allocate Vehicle space for the rotated angle vectors
for i = 1:length(av_att(1, :)) % Loop through each snapshot in time
    EAVI321_I = RotationMatrix321(av_att(:, i)) * av_att(:, i); % Store the rotated angle vector in EAV Instantaneous vector (at i)
    % Putting Instantaneous Rotated Euler Angles into 3 x 1 Vector
    EAV321(1, i) = EAVI321_I(1);
    EAV321(2, i) = EAVI321_I(2);
    EAV321(3, i) = EAVI321_I(3);
end

% EAT321 is Euler Angle of Target Rotated by 3-2-1 Euler Angles (Inertial -> Body)

EAT321 = zeros(3, length(tar_att(1, :))); % Allocate Target space for the rotated angle vectors
for i = 1:length(tar_att(1, :)) % Loop through each snapshot in time
    EATI321_I = RotationMatrix321(tar_att(:, i)) * tar_att(:, i); % Store the rotated angle vector in EAT Instantaneous vector (at i)
    % Putting Instantaneous Rotated Euler Angles into 3 x 1 Vector
    EAT321(1, i) = EATI321_I(1);
    EAT321(2, i) = EATI321_I(2);
    EAT321(3, i) = EATI321_I(3);
end

% Plot each attitude component (phi-theta-psi) individually with respect to time
% for each aircraft

figure();
subplot(3,1,1)
plot(t_vec(:), EAV321(1,:), 'g--', t_vec(:), EAT321(1,:), 'b-')
title("Assignment Part B: Phi Component Euler 3-2-1 Angle of Vehicle & Target Over Time")
xlabel("Time (s)")
ylabel("Phi Euler 3-2-1 Angle (deg)")
legend("Vehicle", "Target")
subplot(3,1,2)
plot(t_vec(:), EAV321(2,:), 'g--', t_vec(:), EAT321(2,:), 'b-')
title("Assignment Part B: Theta Component Euler 3-2-1 Angle of Vehicle & Target Over Time")
xlabel("Time (s)")
ylabel("Theta Euler 3-2-1 Angle (deg)")
legend("Vehicle", "Target")
subplot(3,1,3)
plot(t_vec(:), EAV321(3,:), 'g--', t_vec(:), EAT321(3,:), 'b-')
title("Assignment Part B: Psi Component Euler 3-2-1 Angle of Vehicle & Target Over Time")
xlabel("Time (s)")
ylabel("Psi Euler 3-2-1 Angle (deg)")
legend("Vehicle", "Target")

%% Assignment Part C

% Visualizing the isolated position components as well as the 3-1-3 inertial->body rotation angles of each aircraft over time

% Calculate the rotation angles of each aircraft in the body frame using the imported euler
% angles and an inertial -> body frame 3-1-3 rotation matrix

EAV313 = zeros(3, length(av_att(1, :))); % Allocate Vehicle space for the rotated angle vectors
for i = 1:length(av_att(1, :)) % Loop through each snapshot in time
    EAVI313_I = RotationMatrix313(av_att(:, i)) * av_att(:, i); % Store the rotated angle vector in EAV Vector Instantaneous (at i)
    % Putting Instantaneous Rotated Euler Angles into 3 x 1 Vector
    EAV313(1, i) = EAVI313_I(1);
    EAV313(2, i) = EAVI313_I(2);
    EAV313(3, i) = EAVI313_I(3);
end

EAT313 = zeros(3, length(tar_att(1, :))); % Allocate Target space for the rotated angle vectors
for i = 1:length(tar_att(1, :)) % Loop through each snapshot in time
    EATI313_I = RotationMatrix313(tar_att(:, i)) * tar_att(:, i); % Store the rotated angle vector in EAT Vector Instantaneous (at i)
    % Putting Instantaneous Rotated Euler Angles into 3 x 1 Vector
    EAT313(1, i) = EATI313_I(1);
    EAT313(2, i) = EATI313_I(2);
    EAT313(3, i) = EATI313_I(3);
end

% Plot each attitude component (lowercase_omega-i-uppercase_omega) individually with respect to time
% for each aircraft

figure();
subplot(3,1,1)
plot(t_vec(:), EAV313(1,:), 'g--', t_vec(:), EAT313(1,:), 'b-')
title("Assignment Part C: Lowercase Omega Component Euler 3-1-3 Angle of Vehicle & Target Over Time")
xlabel("Time (s)")
ylabel("Lowercase Omega Euler 3-1-3 Angle (deg)")
legend("Vehicle", "Target")
subplot(3,1,2)
plot(t_vec(:), EAV313(2,:), 'g--', t_vec(:), EAT313(2,:), 'b-')
title("Assignment Part C: i Component Euler 3-1-3 Angle of Vehicle & Target Over Time")
xlabel("Time (s)")
ylabel("i Euler 3-1-3 Angle (deg)")
legend("Vehicle", "Target")
subplot(3,1,3)
plot(t_vec(:), EAV313(3,:), 'g--', t_vec(:), EAT313(3,:), 'b-')
title("Assignment Part C: Uppercase Omega Component Euler 3-1-3 Angle of Vehicle & Target Over Time")
xlabel("Time (s)")
ylabel("Uppercase Omega Euler 3-1-3 Angle (deg)")
legend("Vehicle", "Target")

%% Assigment Part D

% Visualizing the relative position of the Target in each component of position over
% time in the Inertial frame

tar_pos_relV = tar_pos_inert - av_pos_inert; % Position of Target Relative to Vehicle is just Target Position - Vehicle
% Position

% Plot each individual component (X-Y-Z) of the Target's position relative to the Vehicle in Inertial Frame Coordinates with respect to time

figure();
subplot(3,1,1)
plot(t_vec(:), tar_pos_relV(1, :), 'b-')
title("Assignment Part D: X Component Position of Target Relative to Vehicle Over Time")
xlabel("Time (s)")
ylabel("X Inertial Displacement (m)")
subplot(3,1,2)
plot(t_vec(:), tar_pos_relV(2, :), 'b-')
title("Assignment Part D: Y Component Position of Target Relative to Vehicle Over Time")
xlabel("Time (s)")
ylabel("Y Inertial Displacement (m)")
subplot(3,1,3)
plot(t_vec(:), tar_pos_relV(1, :), 'b-')
title("Assignment Part D: Z Component Position of Target Relative to Vehicle Over Time")
xlabel("Time (s)")
ylabel("Z Inertial Displacement (m)")

%% Assignment Part E

% Visualizing the relative position of the Target in each component of position over
% time in the Vehicle frame

tar_pos_relV_coordsV = zeros(3, length(tar_pos_relV(1, :)));  % Allocate relative position space for the rotated position vectors
for i = 1:length(tar_pos_relV(1, :)) % Loop through each snapshot in time
    tar_pos_relV_coordsV_I = RotationMatrix321(av_att(:, i)) * tar_pos_relV(:, i); % Store the rotated relative position vector in TargetPosition_RelativeToV_CoordsV Instantaneous (at i)
    % Putting Instantaneous Rotated Relative Position (Body Frame) Values into 3 x 1 Vector
    tar_pos_relV_coordsV(1, i) = tar_pos_relV_coordsV_I(1);
    tar_pos_relV_coordsV(2, i) = tar_pos_relV_coordsV_I(2);
    tar_pos_relV_coordsV(3, i) = tar_pos_relV_coordsV_I(3);
end

% Plot each individual component (X-Y-Z) of the Target's position relative to the Vehicle in Vehicle Frame coordinates with respect to time

figure();
subplot(3,1,1)
plot(t_vec(:), tar_pos_relV_coordsV(1, :), 'b-')
title("Assignment Part E: X Component Position of Target Relative to Vehicle Over Time")
xlabel("Time (s)")
ylabel("X Body Displacement (m)")
subplot(3,1,2)
plot(t_vec(:), tar_pos_relV_coordsV(2, :), 'b-')
title("Assignment Part E: Y Component Position of Target Relative to Vehicle Over Time")
xlabel("Time (s)")
ylabel("Y Body Displacement (m)")
subplot(3,1,3)
plot(t_vec(:), tar_pos_relV_coordsV(1, :), 'b-')
title("Assignment Part E: Z Component Position of Target Relative to Vehicle Over Time")
xlabel("Time (s)")
ylabel("Z Body Displacement (m)")

%% Assignment Part F

% Visualizing the relative angle of the Target in each 3-2-1 angle component over
% time in the Vehicle frame

DCM_AngleParams = zeros(3, length(tar_att(1, :))); % Allocate space for the direction cosine matrix at each snapshot in time
for i = 1:length(tar_att) % Loop through each snapshot in time
    R4ftar = RotationMatrix321(tar_att(:,i)); % Determine the rotation matrix from the inertial frame to the target
    R4fav = RotationMatrix321(av_att(:,i)); % Determine the rotation matrix from the inertial frame to the vehicle
    % The rotation matrix from the vehicle to the target is equal to the
    % product of the previously calculated matrices (with the Inertial->Vehicle Rotation Matrix "reversed"/transposed)
    DCM_AngleParams(:,i) = EulerAngles321(R4ftar * R4fav'); % Pull/extract the euler angles from the product matrix into the DCM_AngleParams matrix/vector
end

% Plot each individual component (Phi-Theta-Psi) of the Target's rotation angles relative to the Vehicle in Vehicle Frame coordinates with respect to time

figure();
subplot(3,1,1)
plot(t_vec(:), DCM_AngleParams(1,:))
title("Assignment Part F: Phi Component of 3-2-1 DCM of Vehicle to Target Over Time")
xlabel("Time (s)")
ylabel("Phi 3-2-1 DCM Angle (deg)")
subplot(3,1,2)
plot(t_vec(:), DCM_AngleParams(2,:))
title("Assignment Part F: Theta Component of 3-2-1 DCM of Vehicle to Target Over Time")
xlabel("Time (s)")
ylabel("Theta 3-2-1 DCM Angle (deg)")
subplot(3,1,3)
plot(t_vec(:), DCM_AngleParams(3,:))
title("Assignment Part F: Psi Component of 3-2-1 DCM of Vehicle to Target Over Time")
xlabel("Time (s)")
ylabel("Psi 3-2-1 DCM Angle (deg)")
