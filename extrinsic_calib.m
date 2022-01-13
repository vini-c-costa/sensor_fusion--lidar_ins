function [T, PC2_Lidar, PC2_INS] = extrinsic_calib(R_lidar, p_lidar, Lidar_XYZ)
    % Receive data (where is LiDAR related to INS)
    T = [R_lidar p_lidar; 0 0 0 1]; % Entire transformation 4x4

    [row, ~] = size(Lidar_XYZ); % Define the number of rows inside Lidar_XYZ
    PC2_Lidar = cell(row,1); % Create a (row,1) matrix for Lidar Data
    PC2_INS = cell(row,1); % Create a (row,1) matrix for INS Data

    for i = 1:1:row

        [row2, ~] = size(Lidar_XYZ{i,1}(:,:)); % Created row2 to see the size inside each Lidar_XYZ cell
        PC2_Lidar{i,1} = Lidar_XYZ{i,1}(:,:); % Define Lidar_XYZ as a vector inside a single
        PC2_Lidar{i,1}(:,4) = ones(row2,1); % Include a column of ones

        PC2_INS{i,1} = (T*(PC2_Lidar{i,1}(:,:)'))'; % multiply T (4x4) and PC2_Lidar' (4x1), transpose it to keep it correct, and we have the result in PC2_INS

        PC2_Lidar{i,1}(:,4) = []; % Delete 4th col
        PC2_INS{i,1}(:,4) = []; % Delete 4th col
        clear row2 col


    end
end