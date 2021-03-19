function [bag, Lidar, INS] = read_bag(b)
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

    Lidar = open_Lidar(bag, bag.EndTime - bag.StartTime);
    INS = open_INS(bag, bag.EndTime - bag.StartTime);

    % if you want to read the entire bag, comment the two lines above,
    % and use the command below:
    %
    % Lidar = openBagLidar(bag, 5);
    % INS   = openBagUINS (bag, 5);
end