function [] = PrintPlot(plt)
    % Generates predefined plots from the simulation data.
    % This function generates time-history plots of various UVMS data, 
    % such as joint positions, velocities, vehicle states, and activation functions. 
    % It provides visual feedback for analyzing the simulation results.
    
    % Plot joint positions and velocities.
    figure(1);
    subplot(2, 1, 1);
    hplot = plot(plt.t, plt.q); % Joint positions over time.
    set(hplot, 'LineWidth', 1);
    legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7');
    subplot(2, 1, 2);
    hplot = plot(plt.t, plt.q_dot); % Joint velocities over time.
    set(hplot, 'LineWidth', 1);
    legend('qdot_1', 'qdot_2', 'qdot_3', 'qdot_4', 'qdot_5', 'qdot_6', 'qdot_7');
    
    % Plot vehicle position and velocities.
    figure(2);
    subplot(3, 1, 1);
    hplot = plot(plt.t, plt.p); % Vehicle position (x, y, z, roll, pitch, yaw).
    set(hplot, 'LineWidth', 1);
    legend('x', 'y', 'z', 'roll', 'pitch', 'yaw');
    subplot(2, 1, 2);
    hplot = plot(plt.t, plt.p_dot); % Vehicle velocities.
    set(hplot, 'LineWidth', 1);
    legend('xdot', 'ydot', 'zdot', 'omega_x', 'omega_y', 'omega_z');
    
    % Plot activation functions.
    figure(3);
    hplot = plot(plt.t, plt.a(1:7, :)); % Joint limit activations.
    set(hplot, 'LineWidth', 2);
    legend('Ajl_11', 'Ajl_22', 'Ajl_33', 'Ajl_44', 'Ajl_55', 'Ajl_66', 'Ajl_77');
    
    figure(4);
    hplot = plot(plt.t, plt.a(8:9, :)); % Other activation functions.
    set(hplot, 'LineWidth', 2);
    legend('Amu', 'Aha');
end
