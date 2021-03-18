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

%% 2 -> Creating Lidar_concat & Lidar_XYZ

[Lidar_concat, Lidar_XYZ] = explore_Lidar(Lidar);



