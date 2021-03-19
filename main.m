close all;          % close windows
clear variables;    % clean variables
clc;                % clean terminal

profile clear       % clean profile history
profile -memory on  % Bring memory info

%% 1 -> Opening and reading the bag

% Define the bag
b = 'Puck-uINS_2020-07-14-14-43-29.bag';
rosbag info 'Puck-uINS_2020-07-14-14-43-29.bag'

% Read the bag to Lidar-INS format
[bag, Lidar, INS] = read_bag(b);

%% 2 -> Creating Lidar_XYZT & Lidar_XYZ

% It will bring two different data:
%
% -> Lidar_XYZ = [X,Y,Z] info
% -> Lidar_XYZT = [X,Y,Z,T] info
%
% The main skill of the function is to concatenate info
% that used to be separated in Lidar.
[Lidar_XYZT, Lidar_XYZ] = explore_Lidar(Lidar);

%% 3 -> Extrinsic Calibration

% The function will utilize the homogeneous transformation
% from INS to Lidar to create PointCloud values to each point,
% considering rotation and translation from INS to Lidar.
%
% Configuring the homogeneous transformation:

% Rotation (e.g R_lidar = rotx(deg1)*roty(deg2)*rotz(deg3)):
R_lidar = rotx(30)*roty(45)*rotz(180);

% Position (e.g p_lidar = [X, Y, Z]'):
p_lidar = [2 4 1]';

% Function that will create PointCloud values to each point,
% in POV Lidar (PC2_Lidar) and POV INS (PC2_INS)

[T, PC2_Lidar, PC2_INS] = extrinsic_calib(R_lidar, p_lidar, Lidar_XYZ);

%% 4 -> Converting PC2 (ROS) to PC (Computer Vision Toolbox)

% PointCloud2 to PointCloud is one of the main convertions of the
% program, because it makes Computer Vision Toolbox available,
% which wasn't possible the usage before.
[PC_INS, PC_Lidar] = convert_PC2_to_PC(PC2_INS, PC2_Lidar);


%% 5 -> Memory review

profile viewer




