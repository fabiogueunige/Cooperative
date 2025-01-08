function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)
% A define the activation task, so it is connected to the lenght of xdot reference fot each task 

% ActionTransition(taskname, previous, current, mission.phase_time);
%taskname = action we want to activate (actual)
%previous = set of actions of previous task
%current = set of actions of actual task

%% EQUALITY TASK ACTIVATION

if mission.phase == 2
    pandaArm.A.rc = eye(6);
else
    pandaArm.A.rc = zeros(6);
end
pandaArm.ArmL.A.tool = eye(6) * ActionTransition("T", mission.prev_action, mission.current_action, mission.phase_time);
pandaArm.ArmR.A.tool = eye(6) * ActionTransition("T", mission.prev_action, mission.current_action, mission.phase_time);

pandaArm.ArmL.A.target = ActionTransition("CM", mission.prev_action, mission.current_action, mission.phase_time) * eye(6);
pandaArm.ArmR.A.target = ActionTransition("CM", mission.prev_action, mission.current_action, mission.phase_time) * eye(6);

pandaArm.ArmL.A.stop = ActionTransition("S", mission.prev_action, mission.current_action, mission.phase_time) * eye(6);  
pandaArm.ArmR.A.stop = ActionTransition("S", mission.prev_action, mission.current_action, mission.phase_time) * eye(6);  

%% INEQUALITY TASK ACTIVATION

% Minimum Altitude Task ( > 0.15m, 0.05m delta )

pandaArm.ArmL.A.ma(6,6) = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArm.ArmL.wTt(3, 4)); % x = position on z-axis
pandaArm.ArmR.A.ma(6,6) = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArm.ArmR.wTt(3, 4));

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
% matrix with on diagonal all the sigmoids
for k = 1:7
    % set a matix with on diagonal sum of two sigmoid, one for the maximum
    % limit and one for the minimum joints limit
    pandaArm.ArmL.A.jl(k,k) = DecreasingBellShapedFunction(pandaArm.jlmin(k), pandaArm.jlmin(k) + pandaArm.jlmin(k) * 0.1, 0, 1, pandaArm.ArmL.q(k)) ...
                               + IncreasingBellShapedFunction(pandaArm.jlmax(k) - pandaArm.jlmax(k) * 0.1, pandaArm.jlmax(k) , 0,1, pandaArm.ArmL.q(k));
end

for k = 8:14
    r = k -7;
    pandaArm.ArmR.A.jl(k,k) = DecreasingBellShapedFunction(pandaArm.jlmin(r), pandaArm.jlmin(r) + pandaArm.jlmin(r) * 0.1, 0, 1, pandaArm.ArmR.q(r)) ...
                              + IncreasingBellShapedFunction(pandaArm.jlmax(r) - pandaArm.jlmax(r) * 0.1, pandaArm.jlmax(r) , 0,1, pandaArm.ArmR.q(r));
end

end
