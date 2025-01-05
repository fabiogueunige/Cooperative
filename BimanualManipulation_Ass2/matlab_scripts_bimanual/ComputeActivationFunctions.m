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
         pandaArm.A.tool = 1 * ActionTransition("T", mission.actions.go_to.tasks, mission.actions.go_to.tasks, mission.phase_time);
        
    case 2 % Move the object holding it firmly
        pandaArm.A.tool = 1 * ActionTransition("T", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time);%0; % TODO remove after using the action transition functions
        % Rigid Grasp Constraint
        pandaArm.A.rc = 1;% * ActionTransition("RC", mission.actions.go_to.tasks, mission.actions.coop_manip, mission.phase_time);
        
         % Move-To
         pandaArm.A.target = 1 * ActionTransition("TC", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time);
    case 3 % STOP any motion 
        
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
    pandaArm.ArmL.A.jl(k,k) = DecreasingBellShapedFunction(pandaArm.jlmin(k), pandaArm.jlmin(k) ...
        + ((pandaArm.jlmin(k) - pandaArm.ArmL.q(k)) .* 0.1) * pandaArm.jlmin(k), 0, 1, pandaArm.ArmL.wTt(3, 4)) ...
        + IncreasingBellShapedFunction(pandaArm.jlmax(k), pandaArm.jlmax(k) + ... 
        ((pandaArm.jlmax(k) - pandaArm.ArmL.q(k)) .* 0.1) * pandaArm.jlmax(k), 0, ...
        1, pandaArm.ArmL.wTt(3, 4));
end

pandaArm.ArmR.A.jl = zeros(7, 7); % matrix with on diagonal all the sigmoids
for k = 1:7
    % set a matix with on diagonal sum of two sigmoid, one for the maximum
    % limit and one for the minimum joints limit
    pandaArm.ArmR.A.jl(k,k) = DecreasingBellShapedFunction(pandaArm.jlmin(k), pandaArm.jlmin(k) ...
        + ((pandaArm.jlmin(k) - pandaArm.ArmR.q(k)) .* 0.1) * pandaArm.jlmin(k), 0, 1, pandaArm.ArmR.wTt(3, 4)) ...
        + IncreasingBellShapedFunction(pandaArm.jlmax(k), pandaArm.jlmax(k) + ... 
        ((pandaArm.jlmax(k) - pandaArm.ArmR.q(k)) .* 0.1) * pandaArm.jlmax(k), 0, ...
        1, pandaArm.ArmR.wTt(3, 4));
end

end
