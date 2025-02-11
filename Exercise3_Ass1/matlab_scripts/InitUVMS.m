function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';

uvms.rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9; -1.6; -2.9; -2.95; -2.9; -1.65; -2.8];
uvms.jlmax  = [2.9; 1.65; 2.9; 0.01; 2.9; 1.25; 2.8];

% to be computed at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.sensorDistance = 0;

uvms.J.t_a = zeros(6,7);
uvms.J.t_v = zeros(6,7);
uvms.J.t = zeros(6,13);
uvms.J.ma = zeros(1,13);
uvms.J.ha = zeros(2, 13);
uvms.J.jl = zeros(7, 13);
uvms.J.vh = zeros(1, 13);
uvms.J.vp = zeros(3, 13);
uvms.J.va = zeros(1,13);
uvms.J.ac =zeros(2, 13);

uvms.xdot.mu = [];
uvms.xdot.jl = [];
uvms.xdot.ha = [];
uvms.xdot.t = [];
uvms.xdot.va = [];

uvms.A.mu = 0;
uvms.A.ma = 0;
uvms.A.ha = zeros(2);
uvms.A.jl = zeros(7);
uvms.A.vh = 0;
uvms.A.t = zeros(6,6);

end

