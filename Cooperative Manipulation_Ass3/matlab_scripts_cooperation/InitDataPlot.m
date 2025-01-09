function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);
    plt.q2 = zeros(7, maxloops);
    plt.q_dot2 = zeros(7, maxloops);
    
    plt.t2 = NaN; % time at which start phase 2
    plt.t3 = NaN; % time at which start phase 3
    plt.xdot.desired = zeros(6,maxloops); % desired object cartesian velocity  
    plt.xdot.actual = zeros(6,maxloops);  % non-cooperative cartesian velocity arm1
    plt.xdot.actual2 = zeros(6,maxloops); % non-cooperative cartesian velocity arm2
    plt.xdot.fc = zeros(12,maxloops);
    plt.t1Dt2 = zeros(6,maxloops);


end

