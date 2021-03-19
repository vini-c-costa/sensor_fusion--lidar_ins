function [PC_INS, PC_Lidar] = convert_PC2_to_PC(PC2_INS, PC2_Lidar)

    PC_INS = cellfun(@(XYZ) pointCloud(XYZ), PC2_INS, 'Un', 0);
    PC_Lidar = cellfun(@(XYZ) pointCloud(XYZ), PC2_Lidar, 'Un', 0);

end