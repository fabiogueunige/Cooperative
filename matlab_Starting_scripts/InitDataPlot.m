% InitDataPlot
% Preallocates memory for storing simulation data for plotting
function [plt] = InitDataPlot(maxloops)
    plt.t = zeros(1, maxloops); % Time array
    plt.q = zeros(7, maxloops); % Joint positions
    plt.q_dot = zeros(7, maxloops); % Joint velocities

    plt.p = zeros(6, maxloops); % Vehicle position (6D)
    plt.p_dot = zeros(6, maxloops); % Vehicle velocity (6D)

    plt.xdot_jl = zeros(7, maxloops); % Joint limit task reference
    plt.xdot_mu = zeros(1, maxloops); % Minimum altitude task reference
    plt.xdot_t = zeros(6, maxloops); % Tool-frame task reference

    plt.a = zeros(11, maxloops); % Activation matrices for tasks
end
