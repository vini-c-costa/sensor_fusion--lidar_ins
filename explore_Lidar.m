function [Lidar_XYZT, Lidar_XYZ] = explore_Lidar(Lidar)

    Lidar_XYZT = Lidar{1}.XYZT;
    Lidar_XYZ = Lidar{1}.XYZ;
    % Lidar_xyz{:,2} = Lidar{1}.XYZIRT{:,2};

    for i = 2:1:size(Lidar)
    
        Lidar_XYZT = cat(1, Lidar_XYZT, Lidar{i}.XYZT);
        Lidar_XYZ = cat(1, Lidar_XYZ, Lidar{i}.XYZ);
    
    end

end