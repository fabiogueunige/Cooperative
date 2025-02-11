function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% MA vehicle minimum altitude tasks
uvms.A.ma = ActionTransition("MA", uvms.prev_action, uvms.act_action, mission.phase_time) *...
                                    DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.altitude);

% HA horizontal attitude task activation function
uvms.A.ha = zeros(2);
uvms.A.ha(1,1) = IncreasingBellShapedFunction(deg2rad(2), deg2rad(6), 0, 1, abs(uvms.p(4))); % threshold for |roll| at 4° +- 2°
uvms.A.ha(2,2) = IncreasingBellShapedFunction(deg2rad(2), deg2rad(6), 0, 1, abs(uvms.p(5))); % threshold for |pitch| at 4° +- 2°

% JL joint limits task attivation function
uvms.A.jl = zeros(7); % matrix with on diagonal all the sigmoids
for k = 1:7
    % set a matix with on diagonal sum of two sigmoid, one for the maximum
    % limit and one for the minimum joints limit
    uvms.A.jl(k,k) = DecreasingBellShapedFunction(uvms.jlmin(k), uvms.jlmin(k) + uvms.jlmin(k) * 0.1, 0, 1, uvms.q(k)) ...
                        + IncreasingBellShapedFunction(uvms.jlmax(k) - uvms.jlmax(k) * 0.1, uvms.jlmax(k) , 0,1, uvms.q(k));
end

% VH vehicle heading
uvms.A.vh = ActionTransition("VH", uvms.prev_action, uvms.act_action, mission.phase_time) * 1; % equality task

% VP veichle position activation function on x, y, z
uvms.A.vp = eye(3) * ActionTransition("VP", uvms.prev_action, uvms.act_action, mission.phase_time); % equality task

% VP veichle position activation function on x, y, z
uvms.A.va = ActionTransition("VA", uvms.prev_action, uvms.act_action, mission.phase_time); % equality task

% AC attitude control activation function
uvms.A.ac = eye(2) * ActionTransition("AC", uvms.prev_action, uvms.act_action, mission.phase_time) * 1; % equality task;

% RN reachin nodule activatin function
uvms.A.t = eye(6) * ActionTransition("RN", uvms.prev_action, uvms.act_action, mission.phase_time);

end
