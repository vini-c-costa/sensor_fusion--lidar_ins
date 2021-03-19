function [T, PC2_Lidar, PC2_INS] = extrinsic_calib(R_lidar, p_lidar, Lidar_XYZ)

    T = [R_lidar p_lidar; 0 0 0 1];

    [row, ~] = size(Lidar_XYZ);
    PC2_Lidar = cell(row,1);
    PC2_INS = cell(row,1);

    for i = 1:1:row

        [row2, ~] = size(Lidar_XYZ{i,1}(:,:));
        PC2_Lidar{i,1} = Lidar_XYZ{i,1}(:,:);
        PC2_Lidar{i,1}(:,4) = ones(row2,1);

        PC2_INS{i,1} = (T*(PC2_Lidar{i,1}(:,:)'))';

        PC2_Lidar{i,1}(:,4) = [];
        PC2_INS{i,1}(:,4) = [];
        clear row2 col


    end
end