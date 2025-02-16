function [pandaArm] = ComputeTaskReferences(pandaArm,mission, goal)

gain = 0.6;
min_alt = 0.15;
delta = 0.05;

% Compute minimum altitude reference ALWAYS
pandaArm.xdot.alt = ((delta + min_alt) - pandaArm.wTt(3,4)) * [0; 0; 0; 0; 0; gain];
% take the smallest value, that is what matters most
% pandaArm.min_alt = min(alt_L, alt_R); TODO !!!!!

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
pandaArm.xdot.jl = gain .* (((pandaArm.jlmax - pandaArm.jlmin)/2) - pandaArm.q);

switch mission.phase
    case 1
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArm.wTg, pandaArm.wTt);

        pandaArm.xdot.tool = gain * [ang; lin];
        % Limits request velocities
        pandaArm.xdot.tool(1:3) = Saturate(pandaArm.xdot.tool(1:3,:),0.2);
        pandaArm.xdot.tool(4:6) = Saturate(pandaArm.xdot.tool(4:6,:),0.2);    
    case 2
        % Rigid Grasp Constraint
        % pandaArm.xdot.rc = zeros(6,1);
        
        % Object position and orientation task reference
        [ang, lin] = CartError (goal.los, pandaArm.wTo); 
        pandaArm.xdot.tool = gain * [ang; lin];

        % limit the requested velocities...
        pandaArm.xdot.tool(1:3) = Saturate(pandaArm.xdot.tool(1:3,:), 2);
        pandaArm.xdot.tool(4:6) = Saturate(pandaArm.xdot.tool(4:6,:), 2);

    case 3
        % Stop any motions
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.xdot.tool(1:3) = zeros(3, 1);
        pandaArm.xdot.tool(4:6) = zeros(3, 1);
end


