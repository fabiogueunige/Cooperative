function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        pandaArm.A.tool = ActionTransition("T", mission.actions.go_to.tasks, mission.actions.go_to.tasks, mission.phase_time) * eye(6);
    case 2 % Move the object holding it firmly
        
        % Deactivate previous task 
        pandaArm.A.tool = ActionTransition("T", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time) * eye(6); 

        % Activate Rigid Grasp Constraint
        pandaArm.A.rc = ActionTransition("RC", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time);
        
        % Activate Move-To
        pandaArm.A.target = ActionTransition("TC", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time) * eye(6);
       
    case 3 % STOP any motion 

        % Deactivate Move-to
        pandaArm.A.target = ActionTransition("TC", mission.actions.coop_manip.tasks, mission.actions.end_motion.tasks, mission.phase_time) * eye(6);

        % Activate stop motion
        pandaArm.A.stop = ActionTransition ("S", mission.actions.coop_manip.tasks, mission.actions.end_motion.tasks, mission.phase_time) * eye(6);
end

% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArm.A.ma = zeros(6,6);
pandaArm.A.ma(6, 6) = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArm.wTt(3, 4)); % x = position on z-axis

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error

pandaArm.A.jl = zeros(7); % matrix with on diagonal all the sigmoids
for k = 1:7
    % set a matix with on diagonal sum of two sigmoid, one for the maximum
    % limit and one for the minimum joints limit
    pandaArm.A.jl(k,k) = DecreasingBellShapedFunction(pandaArm.jlmin(k), pandaArm.jlmin(k) + pandaArm.jlmin(k) * 0.1, 0, 1, pandaArm.q(k)) ...
                        + IncreasingBellShapedFunction(pandaArm.jlmax(k) - pandaArm.jlmax(k) * 0.1, pandaArm.jlmax(k) , 0,1, pandaArm.q(k));
end
   
end