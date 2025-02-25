% Creazione del grafico
figure;

% Calcolo dell'asse temporale
time = 1:(loop - 1400); % Asse temporale
% Plot delle velocità lineari
subplot(2, 1, 1); % Prima finestra: velocità lineari del braccio sinistro
plot(time, ref_vel_left_x(1:loop-1400), 'r', 'DisplayName', 'v_{x, left}'); hold on;
plot(time, ref_vel_left_y(1:loop-1400), 'g', 'DisplayName', 'v_{y, left}');
plot(time, ref_vel_left_z(1:loop-1400), 'b', 'DisplayName', 'v_{z, left}');
xlabel('Iterations');
ylabel('Linear Speed left arm [m/s]');
legend;
grid on;

% Seconda finestra: velocità lineari del braccio destro
subplot(2, 1, 2);
plot(time, ref_vel_right_x(1:loop-1400), 'r', 'DisplayName', 'v_{x, right}'); hold on
plot(time, ref_vel_right_y(1:loop-1400), 'g', 'DisplayName', 'v_{y, right}');
plot(time, ref_vel_right_z(1:loop-1400), 'b', 'DisplayName', 'v_{z, right}');
xlabel('Iterations');
ylabel('Linear Speed right arm [m/s]');
legend;
grid on;
hold off;



%Definizione dell'asse temporale
time = 1:(loop -1400);
% Creazione della figura
figure;
plot(time, obj_vel_x(1:loop-1400), 'r', 'DisplayName', 'Linear Velocity X'); hold on;
plot(time, obj_vel_y(1:loop-1400), 'g', 'DisplayName', 'Linear Velocity Y');
plot(time, obj_vel_z(1:loop-1400), 'b', 'DisplayName', 'Linear Velocity Z');
xlabel('Iterations');
ylabel('Linear Obj Velocity (m/s)');
title('Cartesian linear obj Velocity');
legend;
grid on;
hold off;

