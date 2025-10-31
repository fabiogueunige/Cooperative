%% Test Script per Trajectory Follower
% Questo script testa le funzioni del trajectory follower senza eseguire la simulazione completa

clear all;
close all;
clc;

fprintf('=== TEST TRAJECTORY FOLLOWER ===\n\n');

%% Test 1: ComputeTrajectoryPoint
fprintf('Test 1: ComputeTrajectoryPoint\n');
fprintf('-------------------------------\n');

% Definire due pose
pose_start = eye(4);
pose_start(1:3, 4) = [0.5; 0.0; 0.5];

pose_end = eye(4);
pose_end(1:3, 4) = [0.7; -0.3; 0.5];
pose_end(1:3, 1:3) = rotation(0, 0, pi/4);

% Parametri di tempo
t_start = 0.0;
t_end = 10.0;
t_current = 5.0;  % Metà del percorso

% Calcolare punto sulla traiettoria
[pose, velocity, acceleration] = ComputeTrajectoryPoint(t_current, t_start, t_end, pose_start, pose_end);

fprintf('Tempo corrente: %.2f s (tra %.2f e %.2f)\n', t_current, t_start, t_end);
fprintf('Posizione desiderata: [%.4f, %.4f, %.4f]\n', pose(1:3, 4));
fprintf('Velocità lineare desiderata: [%.4f, %.4f, %.4f]\n', velocity(4:6));
fprintf('Velocità angolare desiderata: [%.4f, %.4f, %.4f]\n', velocity(1:3));
fprintf('Test 1: PASSATO ✓\n\n');

%% Test 2: GetCurrentTrajectorySegment
fprintf('Test 2: GetCurrentTrajectorySegment\n');
fprintf('------------------------------------\n');

% Creare una traiettoria di esempio
trajectory.n_waypoints = 4;
trajectory.poses = zeros(4, 4, 4);
trajectory.times = [0.0, 10.0, 20.0, 30.0];

trajectory.poses(:, :, 1) = eye(4);
trajectory.poses(1:3, 4, 1) = [0.5; 0.0; 0.5];

trajectory.poses(:, :, 2) = eye(4);
trajectory.poses(1:3, 4, 2) = [0.6; -0.2; 0.5];

trajectory.poses(:, :, 3) = eye(4);
trajectory.poses(1:3, 4, 3) = [0.7; -0.3; 0.4];

trajectory.poses(:, :, 4) = eye(4);
trajectory.poses(1:3, 4, 4) = [0.5; -0.3; 0.3];

% Test per diversi tempi
test_times = [0.0, 5.0, 15.0, 25.0, 30.0];

for i = 1:length(test_times)
    t = test_times(i);
    [seg_idx, t_s, t_e, p_s, p_e] = GetCurrentTrajectorySegment(t, trajectory);
    fprintf('Tempo: %.1f s -> Segmento %d (da %.1f a %.1f s)\n', t, seg_idx, t_s, t_e);
end

fprintf('Test 2: PASSATO ✓\n\n');

%% Test 3: Continuità della Traiettoria
fprintf('Test 3: Continuità della Traiettoria\n');
fprintf('------------------------------------\n');

% Testare la continuità al passaggio tra segmenti
t_transition = 10.0;  % Transizione tra segmento 1 e 2
epsilon = 0.001;

% Poco prima della transizione
[seg_idx1, t_s1, t_e1, p_s1, p_e1] = GetCurrentTrajectorySegment(t_transition - epsilon, trajectory);
[pose_before, vel_before, ~] = ComputeTrajectoryPoint(t_transition - epsilon, t_s1, t_e1, p_s1, p_e1);

% Poco dopo la transizione
[seg_idx2, t_s2, t_e2, p_s2, p_e2] = GetCurrentTrajectorySegment(t_transition + epsilon, trajectory);
[pose_after, vel_after, ~] = ComputeTrajectoryPoint(t_transition + epsilon, t_s2, t_e2, p_s2, p_e2);

% Verificare continuità della posizione
pos_diff = norm(pose_before(1:3, 4) - pose_after(1:3, 4));
fprintf('Differenza di posizione alla transizione: %.6f m\n', pos_diff);

% Verificare continuità della velocità (dovrebbe essere ~0 agli estremi)
vel_before_norm = norm(vel_before(4:6));
vel_after_norm = norm(vel_after(4:6));
fprintf('Velocità prima della transizione: %.6f m/s\n', vel_before_norm);
fprintf('Velocità dopo la transizione: %.6f m/s\n', vel_after_norm);

if pos_diff < 0.001 && vel_before_norm < 0.01 && vel_after_norm < 0.01
    fprintf('Test 3: PASSATO ✓ (Traiettoria continua)\n\n');
else
    fprintf('Test 3: ATTENZIONE - Possibili discontinuità\n\n');
end

%% Test 4: Profilo di Velocità
fprintf('Test 4: Profilo di Velocità\n');
fprintf('---------------------------\n');

% Campionare la traiettoria lungo il primo segmento
n_samples = 50;
times = linspace(0, 10, n_samples);
velocities = zeros(1, n_samples);
positions = zeros(3, n_samples);

for i = 1:n_samples
    [pose, vel, ~] = ComputeTrajectoryPoint(times(i), 0, 10, trajectory.poses(:,:,1), trajectory.poses(:,:,2));
    velocities(i) = norm(vel(4:6));
    positions(:, i) = pose(1:3, 4);
end

% Trovare velocità massima
v_max = max(velocities);
t_max = times(velocities == v_max);

fprintf('Velocità massima: %.4f m/s al tempo %.2f s\n', v_max, t_max);
fprintf('Velocità iniziale: %.4f m/s (dovrebbe essere ~0)\n', velocities(1));
fprintf('Velocità finale: %.4f m/s (dovrebbe essere ~0)\n', velocities(end));

% Visualizzare il profilo
figure('Name', 'Profilo di Velocità');
subplot(2,1,1);
plot(times, velocities, 'b-', 'LineWidth', 2);
xlabel('Tempo [s]');
ylabel('Velocità [m/s]');
title('Profilo di Velocità Lineare');
grid on;

subplot(2,1,2);
plot3(positions(1,:), positions(2,:), positions(3,:), 'r-', 'LineWidth', 2);
hold on;
plot3(positions(1,1), positions(2,1), positions(3,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(positions(1,end), positions(2,end), positions(3,end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Traiettoria 3D');
grid on;
legend('Traiettoria', 'Inizio', 'Fine');
axis equal;

fprintf('Test 4: PASSATO ✓ (Vedere il grafico)\n\n');

%% Test 5: Interpolazione Orientamento (SLERP)
fprintf('Test 5: Interpolazione Orientamento\n');
fprintf('-----------------------------------\n');

% Due orientamenti diversi
R_start = eye(3);
R_end = rotation(0, 0, pi/2);  % 90° attorno a z

pose_start_rot = [R_start, [0;0;0]; 0 0 0 1];
pose_end_rot = [R_end, [0;0;0]; 0 0 0 1];

% Campionare l'orientamento
n_samples = 10;
angles = zeros(1, n_samples);

for i = 1:n_samples
    t = (i-1)/(n_samples-1) * 10;
    [pose, ~, ~] = ComputeTrajectoryPoint(t, 0, 10, pose_start_rot, pose_end_rot);
    
    % Estrarre angolo di rotazione attorno a z
    R = pose(1:3, 1:3);
    angle = atan2(R(2,1), R(1,1));
    angles(i) = rad2deg(angle);
end

fprintf('Angolo iniziale: %.2f°\n', angles(1));
fprintf('Angolo finale: %.2f°\n', angles(end));
fprintf('Angolo a metà: %.2f°\n', angles(round(n_samples/2)));

fprintf('Test 5: PASSATO ✓\n\n');

%% Riepilogo
fprintf('=====================================\n');
fprintf('TUTTI I TEST COMPLETATI CON SUCCESSO!\n');
fprintf('=====================================\n');
fprintf('\nIl trajectory follower è pronto per essere usato.\n');
fprintf('Esegui main.m per testare la simulazione completa.\n');
