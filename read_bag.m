function [bag, Lidar, INS] = read_bag(b, isLidar, isINS)
    % Project: Sensor fusion LiDAR + INS
    % Name: Vinicius Conti da Costa
    %
    % Bringing a bag as entrance, the function gives all the information
    % as output (entire read).
    %
    % Trazendo como entrada a bag, tem-se como saída a leitura completa.
    %
    % Input: b
    % Output: bag, Lidar, INS

    %% Opening the bag

    bag = rosbag(b);

    %% Reading the bag to each sensor
    % Input = (bag, t_end)
    % Outputs = Lidar{} e INS{}
    
    if isLidar
        Lidar = open_Lidar(bag, bag.EndTime - bag.StartTime);
    else
        Lidar = 0;
    end
    
    if isINS
        INS = open_INS(bag, bag.EndTime - bag.StartTime);
    else
        INS = 0;
    end

    % if you want to read just part of the bag, comment
    % the two lines above, and use the command below:
    %
    % Lidar = openBagLidar(bag, 5);
    % INS   = openBagUINS (bag, 5);
end