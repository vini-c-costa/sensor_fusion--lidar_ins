function [bag, Lidar, INS] = read_bag(b, isLidar, isINS)
    % Project: Sensor fusion LiDAR + INS
    % Name: Vinicius Conti da Costa
    %
    % Bringing a bag as entrance, the function gives all the information
    % as output (entire read).
    %
    % Input: b, isLidar, isINS
    % Output: bag, Lidar, INS

    %% Opening the bag

    bag = rosbag(b);

    %% Reading the bag to each sensor
    % Input = (bag, t_end)
    % Outputs = Lidar{} e INS{}
    
    if isLidar
        Lidar = open_Lidar(bag, 2);
    else
        Lidar = 0;
    end
    
    if isINS
        INS = open_INS(bag, 2);
    else
        INS = 0;
    end

    % if you want to read just part of the bag, comment
    % the two lines above, and use the command below:
    %
    % Lidar = open_Lidar(bag, 5);
    % INS   = open_INS (bag, 5);
end