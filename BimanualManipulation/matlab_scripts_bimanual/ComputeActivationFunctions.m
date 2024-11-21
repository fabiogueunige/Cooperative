function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
         pandaArm.A.tool = ...;
    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        
        % Move-To
        
    case 3 % STOP any motion 
        
end
% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArm.A.ma = ...;

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
pandaArm.A.jl = ...;

end
