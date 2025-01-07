function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)
% A define the activation task, so it is connected to the lenght of xdot reference fot each task 

% ActionTransition(taskname, previous, current, mission.phase_time);
%taskname = action we want to activate (actual)
%previous = set of actions of previous task
%current = set of actions of actual task

%% EQUALITY TASK ACTIVATION

switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        pandaArm.A.tool = ActionTransition("T", mission.actions.go_to.tasks, mission.actions.go_to.tasks, mission.phase_time);
        
    case 2 % Move the object holding it firmly
        pandaArm.A.tool = ActionTransition("T", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time);%0; % TODO remove after using the action transition functions
        % Rigid Grasp Constraint
        pandaArm.A.rc = ActionTransition("RC", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time);
        
        % Move-To
        pandaArm.A.target = ActionTransition("CM", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time);

    case 3 % STOP any motion
         % Deactivate Move-to
        pandaArm.A.target = ActionTransition("CM", mission.actions.coop_manip.tasks, mission.actions.end_motion.tasks, mission.phase_time) * eye(6);

        pandaArm.A.stop = ActionTransition("S", mission.actions.coop_manip.tasks, mission.actions.end_motion.tasks, mission.phase_time) * eye(6);    
       
end
%% INEQUALITY TASK ACTIVATION

%% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArm.ArmL.A.ma = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArm.ArmL.wTt(3, 4)); % x = position on z-axis
pandaArm.ArmR.A.ma = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArm.ArmR.wTt(3, 4));

%% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
pandaArm.ArmL.A.jl = zeros(7, 7); % matrix with on diagonal all the sigmoids
for k = 1:7
    % set a matix with on diagonal sum of two sigmoid, one for the maximum
    % limit and one for the minimum joints limit
    pandaArm.ArmL.A.jl(k,k) = DecreasingBellShapedFunction(pandaArm.jlmin(k), ...
        pandaArm.jlmin(k) + pandaArm.jlmin(k) * 0.1, 0, 1, pandaArm.ArmL.q(k)) ...
        + IncreasingBellShapedFunction(pandaArm.jlmax(k) - pandaArm.jlmax(k) * 0.1,   ...
        pandaArm.jlmax(k) , 0,1, pandaArm.ArmL.q(k));
end

pandaArm.ArmR.A.jl = zeros(7, 7); % matrix with on diagonal all the sigmoids
for k = 1:7
    % set a matix with on diagonal sum of two sigmoid, one for the maximum
    % limit and one for the minimum joints limit
    pandaArm.ArmR.A.jl(k,k) = DecreasingBellShapedFunction(pandaArm.jlmin(k), pandaArm.jlmin(k) + pandaArm.jlmin(k) * 0.1, 0, 1, pandaArm.ArmR.q(k)) ...
                        + IncreasingBellShapedFunction(pandaArm.jlmax(k) - pandaArm.jlmax(k) * 0.1, pandaArm.jlmax(k) , 0,1, pandaArm.ArmR.q(k));
 
end

end
