function [uINS_Completo] = open_INS(bag, t2)

k = 1;
N = ceil(t2/60);

uINS_Completo = cell(N, 1);

for T = 1:60:t2
    
     fprintf("Iteração INS: %d\n", k);    
    t = T-1;


    %% Leitura da bag
    bagSel = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/baro');
    uINS.Baro = readMessages(bagSel, 'DataFormat', 'struct'); % Não foi possível abrir sem DataFormat = Struct -> Matlab n tem classe pra armazenar esse tipo
    bagSel = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/gps');
    uINS.GPS = readMessages(bagSel, 'DataFormat', 'struct'); % Não foi possível abrir sem DataFormat = Struct -> Matlab n tem classe pra armazenar esse tipo
    bagSel = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/gps/info');
    uINS.GPS_info = readMessages(bagSel, 'DataFormat', 'struct'); % Não foi possível abrir sem DataFormat = Struct -> Matlab n tem classe pra armazenar esse tipo
    bagSel = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/imu');
    uINS.IMU = readMessages(bagSel);
    bagSel = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/ins');
    uINS.INS = readMessages(bagSel);
    bagSel = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/mag');
    uINS.Mag = readMessages(bagSel, 'DataFormat', 'struct');
    bagSel = select(bag, 'Time', [bag.StartTime+t bag.StartTime+t+60], ...
        'Topic', '/pewinit_imu');
    uINS.PreInit_IMU = readMessages(bagSel);

    %% Leitura dos dados de Tempo
    % Tempo INS (como Referencia)
    time_INS(:, 1) = cellfun(@(m) double(m.Header.Stamp.Sec), uINS.INS); %info tempo maquina (Posix)
    time_INS(:, 2) = cellfun(@(m) double(m.Header.Stamp.Nsec), uINS.INS); %info frame (zera a cada segundo)
    time_INS(:, 3) = time_INS(:, 1) + time_INS(:,2)*1e-9;
    uINS.DateTimeINS = datetime(time_INS(:, 3), ...
        'ConvertFrom','posixtime', ...
        'Format', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');

    % Tempo IMU
    time_IMU(:, 1) = cellfun(@(m) double(m.Header.Stamp.Sec), uINS.IMU); %info tempo maquina (Posix)
    time_IMU(:, 2) = cellfun(@(m) double(m.Header.Stamp.Nsec), uINS.IMU); %info frame (zera a cada segundo)
    time_IMU(:, 3) = time_IMU(:, 1) + time_IMU(:,2)*1e-9;
    DateTime_IMU = datetime(time_IMU(:, 3), ...
        'ConvertFrom','posixtime', ...
        'Format', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');

    % Tempo Magnetômetro
    time_MAG(:, 1) = cellfun(@(m) double(m.Header.Stamp.Sec), uINS.Mag); %info tempo maquina (Posix)
    time_MAG(:, 2) = cellfun(@(m) double(m.Header.Stamp.Nsec), uINS.Mag); %info frame (zera a cada segundo)
    time_MAG(:, 3) = time_MAG(:, 1) + time_MAG(:,2)*1e-9;
    DateTime_MAG = datetime(time_MAG(:, 3), ...
        'ConvertFrom','posixtime', ...
        'Format', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');

    % Tempo GPS
    % time_GPS(:, 1) = cellfun(@(m) double(m.Header.Stamp.Sec), uINS.GPS); %info tempo maquina (Posix)
    % time_GPS(:, 2) = cellfun(@(m) double(m.Header.Stamp.Nsec), uINS.GPS); %info frame (zera a cada segundo)
    % time_GPS(:, 3) = time_GPS(:, 1) + time_GPS(:,2)*1e-9;
    % DateTime_GPS = datetime(time_GPS(:, 3), ...
    %     'ConvertFrom','posixtime', ...
    %     'Format', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');

    %% Lê dados de cada sensor (GPS, IMU, INS)
    % Lê e separa os dados de Accel, Gyro e Mag
    Accels(:,1) = cellfun(@(m) m.LinearAcceleration.X, uINS.IMU);
    Accels(:,2) = cellfun(@(m) m.LinearAcceleration.Y, uINS.IMU);
    Accels(:,3) = cellfun(@(m) m.LinearAcceleration.Z, uINS.IMU);

    Gyros(:,1) = cellfun(@(m) m.AngularVelocity.X, uINS.IMU);
    Gyros(:,2) = cellfun(@(m) m.AngularVelocity.Y, uINS.IMU);
    Gyros(:,3) = cellfun(@(m) m.AngularVelocity.Z, uINS.IMU);

    Mags(:,1) = cellfun(@(m) m.MagneticField.X, uINS.Mag);
    Mags(:,2) = cellfun(@(m) m.MagneticField.Y, uINS.Mag);
    Mags(:,3) = cellfun(@(m) m.MagneticField.Z, uINS.Mag);

    % Lê e separa dados do INS
    Pos_INS(:,1) = cellfun(@(m) m.Pose.Pose.Position.X, uINS.INS);
    Pos_INS(:,2) = cellfun(@(m) m.Pose.Pose.Position.Y, uINS.INS);
    Pos_INS(:,3) = cellfun(@(m) m.Pose.Pose.Position.Z, uINS.INS);
    
    OriQuat_INS(:,1) = cellfun(@(m) m.Pose.Pose.Orientation.W, uINS.INS);
    OriQuat_INS(:,2) = cellfun(@(m) m.Pose.Pose.Orientation.X, uINS.INS);
    OriQuat_INS(:,3) = cellfun(@(m) m.Pose.Pose.Orientation.Y, uINS.INS);
    OriQuat_INS(:,4) = cellfun(@(m) m.Pose.Pose.Orientation.Z, uINS.INS);

    Linear_INS(:,1) = cellfun(@(m) m.Twist.Twist.Linear.X, uINS.INS);
    Linear_INS(:,2) = cellfun(@(m) m.Twist.Twist.Linear.Y, uINS.INS);
    Linear_INS(:,3) = cellfun(@(m) m.Twist.Twist.Linear.Z, uINS.INS);

    Angular_INS(:,1) = cellfun(@(m) m.Twist.Twist.Angular.X, uINS.INS);
    Angular_INS(:,2) = cellfun(@(m) m.Twist.Twist.Angular.Y, uINS.INS);
    Angular_INS(:,3) = cellfun(@(m) m.Twist.Twist.Angular.Z, uINS.INS);

    %% Cria TimeTables para cada dado com timeStamp da INS
    % Timetable do MARG com Accel, Gyro e Mag
    %   Note que a frequência do magnetômetro é menor e que ao sincronizar
    %   haverá repetição de amostras de Mag. E que Algumas informações
    %   anteriores ou posteriores ao tempo de Accel/Gyro podem ser perdidas,
    %   já que estaremos utilizando o tempo de INS
    IMU_tt = timetable(DateTime_IMU, Accels, Gyros);  % Cria a Timetable
    IMU_tt.Properties.VariableNames = {'Accels', 'Gyros'};  % Dá nome às colunas (Pressupõe-se XYZ)

    MAG_tt = timetable(DateTime_MAG, Mags);
    MAG_tt.Properties.VariableNames = {'Mags'};

    uINS.MARG_tt = synchronize(IMU_tt, MAG_tt, uINS.DateTimeINS, 'previous');
% 
%     % TimeTable da INS
%     uINS.INS_tt = timetable(uINS.DateTimeINS, Pos_INS(:,1), Pos_INS(:,2), Pos_INS(:,3), ...
%         OriQuat_INS(:, 1), OriQuat_INS(:, 2), OriQuat_INS(:, 3), OriQuat_INS(:, 4));
%     uINS.INS_tt.Properties.VariableNames = {'PosX', 'PosY', 'PosZ', 'OriW', 'OriX', 'OriY', 'OriZ'};

    %Todos os dados de posicao e orientacao no mesmo lugar

    uINS.XYZT(:,1) = cellfun(@(m) m.Pose.Pose.Position.X, uINS.INS);
    uINS.XYZT(:,2) = cellfun(@(m) m.Pose.Pose.Position.Y, uINS.INS);
    uINS.XYZT(:,3) = cellfun(@(m) m.Pose.Pose.Position.Z, uINS.INS);
    uINS.XYZT(:,4) = time_INS(:,3);
    
    uINS.XYZWXYZT(:,1) = cellfun(@(m) m.Pose.Pose.Position.X, uINS.INS);
    uINS.XYZWXYZT(:,2) = cellfun(@(m) m.Pose.Pose.Position.Y, uINS.INS);
    uINS.XYZWXYZT(:,3) = cellfun(@(m) m.Pose.Pose.Position.Z, uINS.INS);
    uINS.XYZWXYZT(:,4) = cellfun(@(m) m.Pose.Pose.Orientation.W, uINS.INS);
    uINS.XYZWXYZT(:,5) = cellfun(@(m) m.Pose.Pose.Orientation.X, uINS.INS);
    uINS.XYZWXYZT(:,6) = cellfun(@(m) m.Pose.Pose.Orientation.Y, uINS.INS);
    uINS.XYZWXYZT(:,7) = cellfun(@(m) m.Pose.Pose.Orientation.Z, uINS.INS);
    % uINS.XYZWXYZT(:,8) = cellfun(@(m) m.time_MAG(:, 3), uINS);
    % uINS.XYZWXYZ_tt = timetable(uINS.DateTimeINS, uINS.XYZWXYZ);
    % uINS.XYZWXYZ_tt.Properties.VariableNames = {'PosX', 'PosY', 'PosZ', 'OriW', 'OriX', 'OriY', 'OriZ'};
    
    uINS_Completo{k} = uINS;
    k = k + 1;
    clear uINS time_INS time_IMU time_MAG DateTime_IMU ...
        DateTime_MAG IMU_tt MAG_tt Accels Gyros Mags Pos_INS ...
        OriQuat_INS OriEul_INS Linear_INS Angular_INS DateTimeINS;
    
    
    
end

end

%% TODO
% 1) Sincronizar IMU com INS -> Timestamp da INS?
% 2) INS tem "Twist" que provavelmente é Gyro e Accel -> São processados?
% -> Gyro não, Accel é removida a gravidade (LinAccel)
% 3) Falta GPS
% 4) Posição do INS -> GPS ECEF + MARG?
% 