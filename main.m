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

%% 2 -> Creating Lidar_XYZT & Lidar_XYZ

% It will bring two different data:
%
% -> Lidar_XYZ = [X,Y,Z] info
% -> Lidar_XYZT = [X,Y,Z,T] info
%
% The main skill of the function is to concatenate info
% that used to be separated in Lidar.

if isLidar
    [Lidar_XYZT, Lidar_XYZ] = explore_Lidar(Lidar);
end
%% 3 -> Extrinsic Calibration

% The function will utilize the homogeneous transformation
% from INS to Lidar to create PointCloud values to each point,
% considering rotation and translation from INS to Lidar.
%
% Configuring the homogeneous transformation:
if isLidar && isINS

    % Rotation (e.g R_lidar = rotx(deg1)*roty(deg2)*rotz(deg3)):
    R_lidar = rotx(0)*roty(0)*rotz(180);

    % Position (e.g p_lidar = [X, Y, Z]'):
    p_lidar = [0 0 1]';

    % Function that will create PointCloud values to each point,
    % in POV Lidar (PC2_Lidar) and POV INS (PC2_INS)

    [T, PC2_Lidar, PC2_INS] = extrinsic_calib(R_lidar, p_lidar, Lidar_XYZ);

    %% 4 -> Converting PC2 (ROS) to PC (Computer Vision Toolbox)

    % PointCloud2 to PointCloud is one of the main convertions of the
    % program, because it makes Computer Vision Toolbox available,
    % which wasn't possible the usage before.
    [PC_INS, PC_Lidar] = convert_PC2_to_PC(PC2_INS, PC2_Lidar);
end
%% 5 -> Interpolation
if isINS && isLidar
    interpolation = interp_Lidar_INS(Lidar_XYZ, Lidar_XYZT, INS);
end
%% 6 -> Memory review

% profile viewer

%% ENTIRE POSE
[row5, ~] = size(INS{1,1}.XYZWXYZ);
pose_INS = cell(row5, 7);
XYZWXYZ = INS{1,1}.XYZWXYZ;
XYZWXYZ = num2cell(XYZWXYZ);

XYZ = cell(row5, 3);
WXYZ = cell (row5, 4);

%%

for i = 1:1:row5
    XYZ{i,1} = XYZWXYZ{i,1};
    XYZ{i,2} = XYZWXYZ{i,2};
    XYZ{i,3} = XYZWXYZ{i,3};
    
    WXYZ{i,1} = XYZWXYZ{i,4};
    WXYZ{i,2} = XYZWXYZ{i,5};
    WXYZ{i,3} = XYZWXYZ{i,6};
    WXYZ{i,4} = XYZWXYZ{i,7};
end

%%

XYZ_t = XYZ';
XYZ = cell2mat(XYZ);
WXYZ = cell2mat(WXYZ);
row_ones = num2cell(ones(1, row5));

%%

for i = 1:1:row5
    XYZ_t{4,i} = 1;
end
%%

rotation_matrix = cell(row5,1);
temp_quat2rotm = cell(3,3);
transformation_matrix = cell(row5,1);
temp_pos = cell(4,1);
temp_transform = cell(4,4);
corke_transformation = zeros (4,4,row5);

for i = 1:1:row5
    temp_quat2rotm = quat2rotm(WXYZ(i,:));
    temp_quat2rotm = num2cell(temp_quat2rotm);
    for j = 1:1:3
        temp_quat2rotm{4,j} = 0;
    end
    rotation_matrix{i,1} = temp_quat2rotm;
    temp_pos = XYZ_t(:,1);
    temp_pos = cell2mat(temp_pos);
    temp_transform = temp_quat2rotm;
    for j = 1:1:4
        temp_transform{j,4} = temp_pos(j,1);
    end
    
    transformation_matrix{i,1}= cell2mat(temp_transform);
    
    for j = 1:1:4
        for k = 1:1:4
            corke_transformation(:,:,i) = cell2mat(temp_transform);
        end
    end
end


%%
for i = 1:1:row5
    for j = 1:1:7
        pose_INS{i,j} = INS{1,1}.XYZWXYZ(i,j);
    end
end

%% 7.1 -> JUST POSITION Example Scatter Plot (test)

[row3, ~] = size(INS{1,1}.XYZ);
first_pos(1,:) = INS{1,1}.XYZ(1,:);
INS_pos_correction = cell(row3,3);
first_pos = num2cell(first_pos);
INS_XYZ_cell(:,:) = (INS{1,1}.XYZ(:,:));

INS_XYZ_cell = num2cell(INS_XYZ_cell);

INS{1,1}.XYZ(1,1) = 0;
INS{1,1}.XYZ(1,2) = 0;
INS{1,1}.XYZ(1,3) = 0;

% cell_data{i,1} = INS{1,1}.XYZ(:,:);

%% 




%%
for i = 2:1:row3
    
    INS_pos_correction{i,1} = INS_XYZ_cell{i,1}' - first_pos{1}';
    INS_pos_correction{i,2} = INS_XYZ_cell{i,2}' - first_pos{2}';
    INS_pos_correction{i,3} = INS_XYZ_cell{i,3}' - first_pos{3}';
end

INS_pos_correction_mat = cell2mat(INS_pos_correction);

x = INS_pos_correction_mat(:,1);
y = INS_pos_correction_mat(:,2);
z = INS_pos_correction_mat(:,3);    
% 
% figure
% hold on
% plot3(x,y,z)
% plot3(x,y,z)

sz = [row3,3];
varTypes = ["double","double","double"];
varNames = ["x","y","z"];

% INS_pos_correction_table = table('Size', sz, 'VariableTypes', varTypes, 'VariableNames', varNames);

% figure
view (3)
hold on
for i = 1:1:row3
    scatter3(INS_pos_correction{i,1}, INS_pos_correction{i,2}, INS_pos_correction{i,3})
end

an = animatedline;
for i = 1:1:(row3-1)
    addpoints(an,x(i),y(i),z(i));
    drawnow
end

plot3 (x,y,z)

% scatter3(INS_pos_correction_table)

xlabel ('x')
ylabel ('y')
zlabel ('z')
grid on
axis ('equal')

%%
% view
% hold on
% scatter3(INS_pos_correction{2,1}, INS_pos_correction{2,2}, INS_pos_correction{2,3})
% scatter3(INS_pos_correction{3876,1}, INS_pos_correction{3876,2}, INS_pos_correction{3876,3})
% 
% xlabel ('x')
% ylabel ('y')
% zlabel ('z')
% grid on
% axis ('equal')
%%

%% 7 -> Example plots

pc1 = PC_Lidar{50};
pc2 = PC_INS{50};

pc1_denoise = pcdenoise(pc1);
pc2_denoise = pcdenoise(pc2);

subplot(2,2,1);
pcshow(pc1_denoise);
title('PointCloud from LiDAR');
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');

subplot(2,2,2);
pcshow(pc2_denoise);
title('PointCloud from INS');
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');

subplot (2,2,3);
pc_merge = pcmerge(pc1_denoise, pc2_denoise, 0.1);
pcshow(pc_merge);
title('PointCloud merged');
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');

subplot (2,2,4);
pcshowpair(pc1_denoise,pc2_denoise,'VerticalAxis', ...
    'Z','VerticalAxisDir','Up')
title('Difference Between POV LiDAR and INS');
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');

legend('LiDAR','INS');
