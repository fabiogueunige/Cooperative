% Creazione del grafico

% % Calcolo dell'asse temporale
% time = 1:(loop - 1400); % Asse temporale
% % Plot delle velocità lineari
% subplot(2, 1, 1); % Prima finestra: velocità lineari del braccio sinistro
% plot(time, ref_vel_left_x(1:loop-1400), 'r', 'DisplayName', 'v_{x, left}'); hold on;
% plot(time, ref_vel_left_y(1:loop-1400), 'g', 'DisplayName', 'v_{y, left}');
% plot(time, ref_vel_left_z(1:loop-1400), 'b', 'DisplayName', 'v_{z, left}');
% xlabel('Iterations');
% ylabel('Linear Speed left arm [m/s]');
% legend;
% grid on;
% 
% % Seconda finestra: velocità lineari del braccio destro
% subplot(2, 1, 2);
% plot(time, ref_vel_right_x(1:loop-1400), 'r', 'DisplayName', 'v_{x, right}'); hold on
% plot(time, ref_vel_right_y(1:loop-1400), 'g', 'DisplayName', 'v_{y, right}');
% plot(time, ref_vel_right_z(1:loop-1400), 'b', 'DisplayName', 'v_{z, right}');
% xlabel('Iterations');
% ylabel('Linear Speed right arm [m/s]');
% legend;
% grid on;
% hold off;
% 
% 
% 
% %Definizione dell'asse temporale
% time = 1:(loop - 1400);
% % Creazione della figura
% figure;
% plot(time, obj_vel_x(1:loop-1400), 'r', 'DisplayName', 'Linear Velocity X'); hold on;
% plot(time, obj_vel_y(1:loop-1400), 'g', 'DisplayName', 'Linear Velocity Y');
% plot(time, obj_vel_z(1:loop-1400), 'b', 'DisplayName', 'Linear Velocity Z');
% xlabel('Iterations');
% ylabel('Linear Obj Velocity (m/s)');
% title('Cartesian linear obj Velocity');
% legend;
% grid on;
% hold off;

figure;
traiettoria_R = traiettoria_R(:, 2:end);
traiettoria_L = traiettoria_L(:, 2:end);
los_point = los_point(:, 2:end);
plot3(traiettoria_L(1, :), traiettoria_L(2, :), traiettoria_L(3,:) , 'LineWidth', 1);
hold on;
plot3(traiettoria_R(1, :), traiettoria_R(2, :), traiettoria_R(3,:) , 'LineWidth', 1);
hold on
plot3(los_point(1, :), los_point(2, :), los_point(3,:) , 'LineWidth', 1)
hold on
plot3 ( goal.wTog(1, 4, 1),  goal.wTog(2, 4, 1),  goal.wTog(3, 4, 1), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
hold on
plot3 ( goal.wTog(1, 4, 2),  goal.wTog(2, 4, 2),  goal.wTog(3, 4, 2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
plot3 ( goal.wTog(1, 4, 3),  goal.wTog(2, 4, 3),  goal.wTog(3, 4, 3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
plot3 ( goal.wTog(1, 4, 4),  goal.wTog(2, 4, 4),  goal.wTog(3, 4, 4), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
grid on
legend ("lef arm", "right arm", "los");

figure;
grid on
plot (lineare(1,:));
hold on
plot (lineare(2,:));
plot (lineare(3,:));
legend("x error","y error", "z error");

figure;
plot(distance_x, 'LineWidth', 1);
grid on
title ("distance between end effectors");