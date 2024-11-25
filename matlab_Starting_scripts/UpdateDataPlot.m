function [plt] = UpdateDataPlot(plt, uvms, t, loop)
    % Samples and saves variables from the UVMS structure for later plotting.
    
    % Save the current time in the plot data structure.
    plt.t(loop) = t;
    
    % Tool position in the world frame (translation component of wTt).
    plt.toolPos(:, loop) = uvms.wTt(1:3, 4);
    
    % Save joint positions and velocities.
    plt.q(:, loop) = uvms.q;
    plt.q_dot(:, loop) = uvms.q_dot;
    
    % Save vehicle position and velocities.
    plt.p(:, loop) = uvms.p;
    plt.p_dot(:, loop) = uvms.p_dot;
    
    % Save task-related data (if available).
    % Transform end-effector velocities into the world frame.
    plt.xdot_t(:, loop) = blkdiag(uvms.wTv(1:3, 1:3), uvms.wTv(1:3, 1:3)) * uvms.xdot.t;
    
    % Save activation functions (diagonal elements).
    plt.a(1:7, loop) = diag(uvms.A.jl); % Joint limits activation
    plt.a(8, loop) = uvms.A.mu;         % Manipulability activation
    plt.a(9, loop) = uvms.A.ha(1, 1);   % Horizontal alignment activation
    
    % Save tool x and y positions.
    plt.toolx(:, loop) = uvms.wTt(1, 4);
    plt.tooly(:, loop) = uvms.wTt(2, 4);
end
