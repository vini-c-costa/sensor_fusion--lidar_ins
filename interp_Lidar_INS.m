function interpolation = interp_Lidar_INS(Lidar_XYZ, Lidar_XYZT, INS)

[row, ~] = size(Lidar_XYZ);
interpolation = zeros(row,3);

for i = 1:1:row
    
    interpolation(i,:) = interp1(INS{1,1}.XYZT(:,4), INS{1,1}.XYZT(:,1:3), Lidar_XYZT{i,2});

end


end