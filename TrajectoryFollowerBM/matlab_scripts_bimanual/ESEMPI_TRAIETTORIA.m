%% Esempio: Personalizzazione Traiettoria
% Questo script mostra come modificare la traiettoria del trajectory follower

%% ESEMPIO 1: Modificare i Tempi (velocità del movimento)
% In InitRobot.m, modificare:

% Movimento LENTO (più tempo tra waypoint)
goal.trajectory.times(1) = 0.0;
goal.trajectory.times(2) = 20.0;  % 20 secondi per primo segmento
goal.trajectory.times(3) = 45.0;  % 25 secondi per secondo segmento
goal.trajectory.times(4) = 70.0;  % 25 secondi per terzo segmento

% Movimento VELOCE (meno tempo tra waypoint)
goal.trajectory.times(1) = 0.0;
goal.trajectory.times(2) = 8.0;   % 8 secondi per primo segmento
goal.trajectory.times(3) = 15.0;  % 7 secondi per secondo segmento
goal.trajectory.times(4) = 22.0;  % 7 secondi per terzo segmento

%% ESEMPIO 2: Aggiungere più Waypoint
% In InitRobot.m, per aggiungere un 5° waypoint:

goal.trajectory.n_waypoints = 5;  % Cambiare da 4 a 5
goal.trajectory.poses = zeros(4, 4, goal.trajectory.n_waypoints);
goal.trajectory.times = zeros(1, goal.trajectory.n_waypoints);

% Definire le pose
goal.wTog(1:4, 1:4, 5) = eye(4);
goal.wTog(1:3, 4, 5) = [0.60, -0.20, 0.40]';  % Nuovo waypoint

for i = 1:goal.trajectory.n_waypoints
    goal.trajectory.poses(:, :, i) = goal.wTog(:, :, i);
end

% Definire i tempi
goal.trajectory.times(1) = 0.0;
goal.trajectory.times(2) = 15.0;
goal.trajectory.times(3) = 30.0;
goal.trajectory.times(4) = 45.0;
goal.trajectory.times(5) = 60.0;  % Nuovo tempo

%% ESEMPIO 3: Traiettoria Circolare
% Creare waypoint lungo un cerchio

n_points = 8;  % 8 waypoint lungo il cerchio
goal.trajectory.n_waypoints = n_points;
goal.trajectory.poses = zeros(4, 4, n_points);
goal.trajectory.times = zeros(1, n_points);

% Parametri del cerchio
center = [0.5, -0.3, 0.4]';
radius = 0.15;
total_time = 40.0;  % 40 secondi per completare il cerchio

for i = 1:n_points
    % Angolo per questo waypoint
    theta = 2*pi*(i-1)/(n_points-1);
    
    % Posizione sul cerchio
    x = center(1) + radius*cos(theta);
    y = center(2) + radius*sin(theta);
    z = center(3);
    
    % Creare la pose
    goal.trajectory.poses(:, :, i) = eye(4);
    goal.trajectory.poses(1:3, 4, i) = [x; y; z];
    
    % Tempo uniforme
    goal.trajectory.times(i) = total_time * (i-1)/(n_points-1);
end

%% ESEMPIO 4: Velocità Variabile (segmenti a velocità diverse)
% Primo segmento lento, secondo veloce, terzo medio

goal.trajectory.times(1) = 0.0;
goal.trajectory.times(2) = 20.0;  % 20s - LENTO
goal.trajectory.times(3) = 25.0;  % 5s - VELOCE
goal.trajectory.times(4) = 40.0;  % 15s - MEDIO

%% ESEMPIO 5: Aggiungere Orientamento alla Traiettoria
% Modificare l'orientamento di ogni waypoint

% Waypoint 1: Orientamento iniziale
goal.trajectory.poses(1:3, 1:3, 1) = rotation(0, 0, 0);

% Waypoint 2: Ruota di 45° attorno a z
goal.trajectory.poses(1:3, 1:3, 2) = rotation(0, 0, pi/4);

% Waypoint 3: Ruota di 90° attorno a z
goal.trajectory.poses(1:3, 1:3, 3) = rotation(0, 0, pi/2);

% Waypoint 4: Ritorna a orientamento iniziale
goal.trajectory.poses(1:3, 1:3, 4) = rotation(0, 0, 0);

%% ESEMPIO 6: Regolare il Guadagno di Feedback
% In ComputeTaskReferences.m, modificare gain_obj

% Per tracking più aggressivo (correzione rapida)
gain_obj = 1.2;

% Per tracking più morbido (meno oscillazioni)
gain_obj = 0.5;

% Valore di default
gain_obj = 0.8;

%% ESEMPIO 7: Traiettoria con Pausa
% Fermarsi in un waypoint per alcuni secondi

goal.trajectory.times(1) = 0.0;
goal.trajectory.times(2) = 15.0;  % Raggiunge waypoint 2 a 15s
goal.trajectory.times(3) = 25.0;  % Parte verso waypoint 3 a 25s (pausa di 10s)
goal.trajectory.times(4) = 40.0;

% Per implementare la pausa, duplicare il waypoint:
goal.trajectory.n_waypoints = 5;
goal.trajectory.poses(:, :, 1) = goal.wTog(:, :, 1);
goal.trajectory.poses(:, :, 2) = goal.wTog(:, :, 2);
goal.trajectory.poses(:, :, 3) = goal.wTog(:, :, 2);  % DUPLICATO - pausa qui
goal.trajectory.poses(:, :, 4) = goal.wTog(:, :, 3);
goal.trajectory.poses(:, :, 5) = goal.wTog(:, :, 4);

goal.trajectory.times = [0, 15, 25, 40, 55];  % Pausa tra 15s e 25s

%% ESEMPIO 8: Calcolare Tempi Automaticamente da Velocità Desiderata
% Calcolare i tempi in base a una velocità lineare massima

v_max = 0.05;  % 5 cm/s di velocità massima

% Calcolare distanze tra waypoint consecutivi
for i = 1:(goal.trajectory.n_waypoints - 1)
    p1 = goal.trajectory.poses(1:3, 4, i);
    p2 = goal.trajectory.poses(1:3, 4, i+1);
    distance = norm(p2 - p1);
    
    % Tempo necessario = distanza / velocità
    if i == 1
        goal.trajectory.times(i) = 0.0;
        goal.trajectory.times(i+1) = distance / v_max;
    else
        goal.trajectory.times(i+1) = goal.trajectory.times(i) + distance / v_max;
    end
end

%% ESEMPIO 9: Traiettoria con Accelerazione Limitata
% Per movimenti molto fluidi, aumentare la durata dei segmenti
% e usare profili di velocità più morbidi

% Moltiplicatore per rendere tutto più lento e fluido
smoothness_factor = 1.5;

goal.trajectory.times = goal.trajectory.times * smoothness_factor;

%% ESEMPIO 10: Debug e Visualizzazione
% Aggiungere questo codice in main.m per visualizzare la traiettoria desiderata

if mission.phase == 2 && mod(t, 1.0) == 0  % Ogni secondo
    fprintf('Time: %.2f, Segment: %d/%d\n', ...
        goal.trajectory.time_in_phase, ...
        goal.trajectory.current_segment, ...
        goal.trajectory.n_waypoints - 1);
    
    if isfield(goal.trajectory, 'pose_desired')
        fprintf('Desired pos: [%.3f, %.3f, %.3f]\n', ...
            goal.trajectory.pose_desired(1:3, 4));
        fprintf('Current pos: [%.3f, %.3f, %.3f]\n', ...
            pandaArm.ArmL.wTo(1:3, 4));
    end
end

%% NOTA IMPORTANTE
% Dopo aver modificato la traiettoria in InitRobot.m, 
% ricorda di:
% 1. Salvare il file
% 2. Eseguire "clear all" in MATLAB
% 3. Rieseguire main.m
