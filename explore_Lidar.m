function [Lidar_concat, Lidar_XYZ] = explore_Lidar(Lidar)

    Lidar_concat = Lidar{1}.XYZT;
    Lidar_XYZ = Lidar{1}.XYZ;
    % Lidar_xyz{:,2} = Lidar{1}.XYZIRT{:,2};

    for i = 2:1:size(Lidar)
    
        Lidar_concat = cat(1, Lidar_concat, Lidar{i}.XYZT);
        Lidar_XYZ = cat(1, Lidar_XYZ, Lidar{i}.XYZ);
    
    end

end