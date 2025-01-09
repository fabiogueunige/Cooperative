function [ ] = PrintPlot( plt)

% some predefined plots
% you can add your own
%% q and qdot
fig = figure('Name', 'Joint position and velocity Left Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
title('LEFT ARM');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');

fig = figure('Name', 'Joint position and velocity Right Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q2);
title('RIGHT ARM');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot2);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');

%% Desired vs Actual object velocities
% linear velocities
fig = figure('Name', 'Desired vs Actual Object Linear Velocity, LEFT ARM.');
subplot(2,2,1);
hplot = plot(plt.t, plt.xdot.desired(1,:), 'b-', plt.t, plt.xdot.actual(1,:),'r');
hold on;
if ~isnan(plt.t2) && ~isnan(plt.t3)
    xline(plt.t2, 'k--', 'LineWidth', 1);
    xline(plt.t3, 'k--', 'LineWidth', 1);
end
hold off;
xlabel('Time (s)');
ylabel('x component of xdot (m/s)');
title('des. vs actual xdot');
set(hplot, 'LineWidth', 1);
legend('xdot desired','xdot actual');

subplot(2,2,2);
hplot = plot(plt.t, plt.xdot.desired(2,:), 'b-', plt.t, plt.xdot.actual(2,:),'r');
if ~isnan(plt.t3)
    xline(plt.t2, 'k--', 'LineWidth', 1);
    xline(plt.t3, 'k--', 'LineWidth', 1);
end
hold off;
xlabel('Time (s)');
ylabel('y component of xdot (m/s)');
title('des. vs actual xdot');
set(hplot, 'LineWidth', 1);
legend('xdot desired','xdot actual');

subplot(2,2,3);
hplot = plot(plt.t, plt.xdot.desired(3,:), 'b-', plt.t, plt.xdot.actual(3,:),'r');
hold on;
if ~isnan(plt.t2)
    xline(plt.t2, 'k--', 'LineWidth', 1);
end
if ~isnan(plt.t3)
    xline(plt.t3, 'k--', 'LineWidth', 1);
end
hold off;
xlabel('Time (s)');
ylabel('z component of xdot (m/s)');
title('des. vs actual xdot');
set(hplot, 'LineWidth', 1);
legend('xdot desired','xdot actual');

% angular velocities
fig = figure('Name', 'Desired vs Actual Object Angular Velocity, LEFT ARM.');
subplot(2,2,1);
hplot = plot(plt.t, plt.xdot.desired(4,:), 'b-', plt.t, plt.xdot.actual(4,:),'r');
hold on;
if ~isnan(plt.t2) && ~isnan(plt.t3)
    xline(plt.t2, 'k--', 'LineWidth', 1);
    xline(plt.t3, 'k--', 'LineWidth', 1);
end
hold off;
xlabel('Time (s)');
ylabel('x component of xdot (rad/s)');
title('des. vs actual xdot');
set(hplot, 'LineWidth', 1);
legend('xdot desired','xdot actual');

subplot(2,2,2);
hplot = plot(plt.t, plt.xdot.desired(5,:), 'b-', plt.t, plt.xdot.actual(5,:),'r');
if ~isnan(plt.t3)
    xline(plt.t2, 'k--', 'LineWidth', 1);
    xline(plt.t3, 'k--', 'LineWidth', 1);
end
hold off;
xlabel('Time (s)');
ylabel('y component of xdot (rad/s)');
title('des. vs actual xdot');
set(hplot, 'LineWidth', 1);
legend('xdot desired','xdot actual');

subplot(2,2,3);
hplot = plot(plt.t, plt.xdot.desired(6,:), 'b-', plt.t, plt.xdot.actual(6,:),'r');
hold on;
if ~isnan(plt.t2)
    xline(plt.t2, 'k--', 'LineWidth', 1);
end
if ~isnan(plt.t3)
    xline(plt.t3, 'k--', 'LineWidth', 1);
end
hold off;
xlabel('Time (s)');
ylabel('z component of xdot (rad/s)');
title('des. vs actual xdot');
set(hplot, 'LineWidth', 1);
legend('xdot desired','xdot actual');

%%  cooperative velocities of the two end-effectors
fig = figure('Name', 'Cooperative velocities of the two end-effectors');
subplot(2,1,1);
hplot = plot(plt.t, plt.xdot.fc(1:3,:),plt.t, plt.xdot.fc(7:9,:));
title('Linear Cooperative velocities');
set(hplot, 'LineWidth', 1);
legend('xdot-x_1','xdot-y_1','xdot-z_1','xdot-x_2','xdot-y_2','xdot-z_2');
subplot(2,1,2);
hplot = plot(plt.t, plt.xdot.fc(4:6,:),plt.t, plt.xdot.fc(10:12,:));
title('Angular Cooperative velocities');
set(hplot, 'LineWidth', 1);
legend('xdot-x_1','xdot-y_1','xdot-z_1','xdot-x_2','xdot-y_2','xdot-z_2');

%% Distance between tool of arm1 and tool of arm2
fig = figure('Name', 'Linear distance between tool of arm1 and tool of arm2');
% linear
hplot = plot(plt.t, plt.t1Dt2(1:3,:)); 
hold on;
if ~isnan(plt.t2)
    hline1 = xline(plt.t2, 'k--', 'LineWidth', 1);
else
    hline1 = []; 
end
if ~isnan(plt.t3)
    hline2 = xline(plt.t3, 'k--', 'LineWidth', 1);
else
    hline2 = [];
end
hold off;
xlabel('Time (s)');
ylabel('distance (m)');
title('Linear distance between the two tools');
set(hplot, 'LineWidth', 1);
legend_items = {'x', 'y', 'z'};
if ~isempty(hline1)
    legend_items{end+1} = 'Phase 1 to Phase 2';
end
if ~isempty(hline2)
    legend_items{end+1} = 'Phase 2 to Phase 3';
end
legend([hplot; hline1; hline2], legend_items, 'Location', 'best');
% angular
fig = figure('Name', 'Angular distance between tool of arm1 and tool of arm2');
hplot = plot(plt.t, plt.t1Dt2(4:6,:)); 
hold on;
if ~isnan(plt.t2)
    hline1 = xline(plt.t2, 'k--', 'LineWidth', 1);
else
    hline1 = []; 
end
if ~isnan(plt.t3)
    hline2 = xline(plt.t3, 'k--', 'LineWidth', 1);
else
    hline2 = [];
end
hold off;
title('Angular distance between the two tools');
set(hplot, 'LineWidth', 1);
legend_items = {'x', 'y', 'z'};
if ~isempty(hline1)
    legend_items{end+1} = 'Phase 1 to Phase 2';
end
if ~isempty(hline2)
    legend_items{end+1} = 'Phase 2 to Phase 3';
end
legend([hplot; hline1; hline2], legend_items, 'Location', 'best');



%plot(tempo, A, 'b-', tempo, B, 'r--', 'LineWidth', 1.5);

% ... 
end

