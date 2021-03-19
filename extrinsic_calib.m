function [T, PC_Lidar, PC_INS] = extrinsic_calib(R_lidar, p_lidar, Lidar_XYZ)

T = [R_lidar p_lidar; 0 0 0 1];

[row, ~] = size(Lidar_XYZ);
PC_Lidar = cell(row,1);
PC_INS = cell(row,1);

for i = 1:1:row
    
    [row2, ~] = size(Lidar_XYZ{i,1}(:,:));
    PC_Lidar{i,1} = Lidar_XYZ{i,1}(:,:);
    PC_Lidar{i,1}(:,4) = ones(row2,1);
    
    PC_INS{i,1} = (T*(PC_Lidar{i,1}(:,:)'))';
    
    PC_Lidar{i,1}(:,4) = [];
    PC_INS{i,1}(:,4) = [];
    clear row2 col
    
    
%     matrizpontoslidar
%     matrizpontosins = (matrizH * matrizpontoslidar')'
%     matrizpontosins(:,4) = [];

end
end