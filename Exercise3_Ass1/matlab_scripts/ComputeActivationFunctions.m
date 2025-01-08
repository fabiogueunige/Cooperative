function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
uvms.A.t = eye(6) * ActionTransition("RN", uvms.prev_action, uvms.act_action, mission.phase_time);

% Activation position control on x, y, z
uvms.A.gv = eye(3) * ActionTransition("VP", uvms.prev_action, uvms.act_action, mission.phase_time); % equality task
% Activation position control on x and y
uvms.A.gv2 = eye(2) * ActionTransition("VP2", uvms.prev_action,uvms.act_action, mission.phase_time); % equality task

% vehicle heading
uvms.A.vh = ActionTransition("VH", uvms.prev_action, uvms.act_action, mission.phase_time) * 1; % equality task
uvms.A.vh2 = ActionTransition("VH2", uvms.prev_action,uvms.act_action, mission.phase_time) * 1; % equality tasks

% % vehicle minimum altitude tasks
uvms.A.ma = ActionTransition("MA", uvms.prev_action, uvms.act_action, mission.phase_time) *...
                                    DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.altitude);

% horizontal attitude task activation function
uvms.A.ha = zeros(2);
uvms.A.ha(1,1) = IncreasingBellShapedFunction(deg2rad(2), deg2rad(6), 0, 1, abs(uvms.p(4))); % threshold for |roll| at 4° +- 2°
uvms.A.ha(2,2) = IncreasingBellShapedFunction(deg2rad(2), deg2rad(6), 0, 1, abs(uvms.p(5))); % threshold for |pitch| at 4° +- 2°

% joint limits task attivation function
uvms.A.jl = zeros(7); % matrix with on diagonal all the sigmoids
for k = 1:7
    % set a matix with on diagonal sum of two sigmoid, one for the maximum
    % limit and one for the minimum joints limit
    uvms.A.jl(k,k) = DecreasingBellShapedFunction(uvms.jlmin(k), uvms.jlmin(k) + uvms.jlmin(k) * 0.1, 0, 1, uvms.q(k)) ...
                        + IncreasingBellShapedFunction(uvms.jlmax(k) - uvms.jlmax(k) * 0.1, uvms.jlmax(k) , 0,1, uvms.q(k));
end

% attitude control activation function
uvms.A.ac = eye(2) * ActionTransition("AC", uvms.prev_action, uvms.act_action, mission.phase_time) * 1; % equality task;

% altitude control to 0
uvms.A.acl = ActionTransition("ACL", uvms.prev_action, uvms.act_action, mission.phase_time) * 1; % equality task;