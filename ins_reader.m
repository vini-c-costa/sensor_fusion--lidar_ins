close all;          % close windows
clear variables;    % clean variables
clc;                % clean terminal

profile clear       % clean profile history
profile -memory on  % Bring memory info

%% 1 -> Opening and reading the bag

% Define the bag
b = 'bag_InertialSense_2022_01_14.bag';
rosbag info 'bag_InertialSense_2022_01_14.bag'

isLidar = false;
isINS = true;

% Read the bag to Lidar-INS format
[bag, Lidar, INS] = read_bag(b, isLidar, isINS);

%% 2 -> Defining pose

[row5, ~] = size(INS{1,1}.XYZWXYZ);
pose_INS = cell(row5, 7);
XYZWXYZ = INS{1,1}.XYZWXYZ;
XYZWXYZ = num2cell(XYZWXYZ);

XYZ = cell(row5, 3);
WXYZ = cell (row5, 4);

