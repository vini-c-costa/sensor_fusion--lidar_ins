function [Lidar_Completo] = open_Lidar(bag,t2)
% Leitura dos parâmetros

k = 1;
N = ceil(t2/60);

Lidar_Completo = cell(N, 1);

for T = 1:60:t2
    fprintf("Iteração Lidar: %d\n", k);    
    t = T-1;
    
    bag_Lidar = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/velodyne_nodelet_manager_driver/parameter_descriptions');
    Lidar.Param_driver_desc    = readMessages(bag_Lidar);

    bag_Lidar = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/velodyne_nodelet_manager_driver/parameter_updates');
    Lidar.Param_driver_updt    = readMessages(bag_Lidar);

    bag_Lidar = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/velodyne_nodelet_manager_laserscan/parameter_descriptions');
    Lidar.Param_laserscan_desc = readMessages(bag_Lidar);

    bag_Lidar = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/velodyne_nodelet_manager_laserscan/parameter_updates');
    Lidar.Param_laserscan_updt = readMessages(bag_Lidar);

    bag_Lidar = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/velodyne_nodelet_manager_transform/parameter_descriptions');
    Lidar.Param_transform_desc = readMessages(bag_Lidar);

    bag_Lidar = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/velodyne_nodelet_manager_transform/parameter_updates');
    Lidar.Param_transform_updt = readMessages(bag_Lidar);

    bag_Lidar = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/diagnostics');
    Lidar.Param_diagnostics = readMessages(bag_Lidar);

    % Seleciona tópico da Bag
    bag_Lidar = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
                            'Topic', '/velodyne_points');

    % clear('bag');

    % Exploração Lidar
    Lidar_msg = readMessages(bag_Lidar); % Lê o tópico da Bag com tipo original

    % Dados de interesse 1
    % Dados de tempo global
    Lidar.time(:, 1) = cellfun(@(m) m.Header.Stamp.Sec, Lidar_msg); %info tempo maquina (Posix)
    Lidar.time(:, 2) = cellfun(@(m) m.Header.Stamp.Nsec, Lidar_msg); %info frame (zera a cada segundo)
    Lidar.time(:, 3) = Lidar.time(:, 1) + Lidar.time(:,2)/1e9;

    % Convertendo posix para Datetime
    Lidar.DateTime = datetime(Lidar.time(:, 3), ...
        'ConvertFrom','posixtime', ...
        'Format', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');

    % Dados de interesse 2
    % Intensidade, canal do emissor e tempo interno do LiDAR
    Lidar.XYZ = cellfun(@(m) readXYZ(m), Lidar_msg, 'UniformOutput', false);
    Lidar.Intensity = cellfun(@(m) readField(m, 'intensity'), Lidar_msg, 'UniformOutput', false);
    Lidar.Ring = cellfun(@(m) readField(m, 'ring'), Lidar_msg, 'UniformOutput', false);
    Lidar.Time = cellfun(@(m) readField(m, 'time'), Lidar_msg, 'UniformOutput', false);

    % Todos os seis valores (XYZIRT) em uma única tabela
    Lidar.XYZIRT = [cellfun(@(XYZ,Int,Ring,Time) horzcat(XYZ,Int,Ring,Time),...
        cellfun(@(m) double(m), Lidar.XYZ, 'Un', 0), ...
        cellfun(@(m) double(m), Lidar.Intensity, 'Un', 0), ...
        cellfun(@(m) double(m), Lidar.Ring, 'Un', 0),...
        cellfun(@(m) double(m), Lidar.Time, 'Un', 0), ...
        'UniformOutput', false) ...
        num2cell(Lidar.time(:,3))];

        Lidar.XYZT = [cellfun(@(XYZ,Time) horzcat(XYZ,Time),...
        cellfun(@(m) double(m), Lidar.XYZ, 'Un', 0), ...
        cellfun(@(m) double(m), Lidar.Time, 'Un', 0), ...
        'UniformOutput', false) ...
        num2cell(Lidar.time(:,3))];
    
    
    % Tempo para cada ponto
    for i = 1:length(Lidar.XYZIRT)
        Lidar.XYZIRT{i,1}(:, 7) = Lidar.XYZIRT{i,2} + Lidar.XYZIRT{i,1}(:, 6);
    end

%     Lidar.XYZIRT_tt = timetable(Lidar.DateTime, Lidar.XYZIRT);
%     Lidar.XYZIRT_tt.Properties.VariableNames = {'XYZIRT'};
%     
    Lidar_Completo{k} = Lidar;
    k = k + 1;
    clear Lidar;
end

end