function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
    % Compute distance between tools for plotting
    pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));
    % Compute minimum altitude reference ALWAYS = gain ((min_alt + delta) - altitude)
    gain = 0.2; % our choice (constant)
    delta = 0.05;
    min_alt = 0.15; % giarda se vanno definiti fuori
    pandaArm.ArmL.xdot.alt = gain * ((delta + min_alt) - pandaArm.ArmL.wTt(3,4));
    pandaArm.ArmR.xdot.alt = gain * ((delta + min_alt) - pandaArm.ArmR.wTt(3,4));
    
    % Compute joint limits task reference ALWAYS
    % Create a velocity away from the limits => move to the middle between jlmax and jlmin

    % joint limits corresponding to the actual Panda by Franka arm configuration
    % (preso da init robot), controlla -> delta is 10% of jl - pos Joint
    jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
    jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

    pandaArm.ArmL.xdot.jl = gain .* (jlmax + ((jlmax - pandaArm.ArmL.q) .* 0.1) - pandaArm.ArmL.q);
    pandaArm.ArmR.xdot.jl = gain .* (jlmax + ((jlmax - pandaArm.ArmR.q) .* 0.1) - pandaArm.ArmR.q);
    
    switch mission.phase
        case 1 
            % Reach the grasping point
            % LEFT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            [ang, lin] = CartError(pandaArms.ArmL.wTt, pandaArms.ArmL.wTg); % e.g. CartError(wTg, wTv) returns the error that makes <v> -> <g>
           
            pandaArm.ArmL.xdot.tool = gain * [ang, lin];
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
end

