function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Compute distance between tools for plotting
pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));
% Compute minimum altitude reference ALWAYS

pandaArm.ArmL.xdot.alt = ...;
pandaArm.ArmR.xdot.alt = ...;

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin

pandaArm.ArmL.xdot.jl = ...;
pandaArm.ArmR.xdot.jl = ...;

switch mission.phase
    case 1
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArms.ArmL.wTt, pandaArms.ArmL.wTg); % e.g. CartError(wTg, wTv) returns the error that makes <v> -> <g>
       
        pandaArm.ArmL.xdot.tool = ...;
        % limit the requested velocities...
        pandaArm.ArmL.xdot.tool(1:3) = Saturate();
        pandaArm.ArmL.xdot.tool(4:6) = Saturate();

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArms.ArmR.wTt, pandaArms.ArmR.wTg);
       
        pandaArm.ArmR.xdot.tool = ...;
        % limit the requested velocities...
        pandaArm.ArmR.xdot.tool(1:3) = Saturate();
        pandaArm.ArmR.xdot.tool(4:6) = Saturate();
    case 2
        % Perform the rigid grasp of the object and move it

        % COMMON
        % -----------------------------------------------------------------
        % Rigid Grasp Constraint
        pandaArm.xdot.rc = ...;

        % LEFT ARM
        % -----------------------------------------------------------------        
        % Object position and orientation task reference
        [ang, lin] = CartError();
        pandaArm.ArmL.xdot.tool = ...;
        % limit the requested velocities...
        pandaArm.ArmL.xdot.tool(1:3) = Saturate();
        pandaArm.ArmL.xdot.tool(4:6) = Saturate();

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Object position and orientation task reference
        [ang, lin] = CartError();
        pandaArm.ArmR.xdot.tool = ...;
        % limit the requested velocities...
        pandaArm.ArmR.xdot.tool(1:3) = Saturate();
        pandaArm.ArmR.xdot.tool(4:6) = Saturate();
    case 3
        % Stop any motions
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.ArmL.xdot.tool(1:3) = ...;
        pandaArm.ArmL.xdot.tool(4:6) = ...;

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.ArmR.xdot.tool(1:3) = ...;
        pandaArm.ArmR.xdot.tool(4:6) = ...;
end


