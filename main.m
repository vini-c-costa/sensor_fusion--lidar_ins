close all;          % close windows
clear variables;    % clean variables
clc;                % clean terminal

profile clear       % clean profile history
profile -memory on  % Bring memory info

%% 1 -Opening and reading the bag

% Function that do the first part of the code

% Selecting the bag

b = 'Puck-uINS_2020-07-14-14-43-29.bag';
rosbag info 'Puck-uINS_2020-07-14-14-43-29.bag'

[bag, Lidar, INS] = read_bag(b);