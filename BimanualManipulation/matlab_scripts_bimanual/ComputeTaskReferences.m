function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
    % Compute distance between tools for plotting
    pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));

    %% Compute minimum altitude reference ALWAYS = gain ((min_alt + delta) - altitude)
    gain = 0.2; % our choice (constant)
    delta = 0.05;
    min_alt = 0.15; % giarda se vanno definiti fuori
    

    pandaArm.ArmL.xdot.alt = ((delta + min_alt) - pandaArm.ArmL.wTt(3,4)) * [0; 0; 0; 0; 0; gain];% generate a positive velocity, according with x-axis, before minimum altitude task is inactive
    pandaArm.ArmR.xdot.alt = ((delta + min_alt) - pandaArm.ArmR.wTt(3,4)) * [0; 0; 0; 0; 0; gain];
    
    %% Compute joint limits task reference ALWAYS
    % Create a velocity away from the limits => move to the middle between jlmax and jlmin

    % joint limits corresponding to the actual Panda by Franka arm configuration
    % (preso da init robot), controlla -> delta is 10% of jl - pos Joint

    pandaArm.ArmL.xdot.jl.max = gain .* (pandaArm.jlmax + ((pandaArm.jlmax - pandaArm.ArmL.q) .* 0.1) - pandaArm.ArmL.q);
    pandaArm.ArmR.xdot.jl.max = gain .* (pandaArm.jlmax + ((pandaArm.jlmax - pandaArm.ArmR.q) .* 0.1) - pandaArm.ArmR.q);
    
    pandaArm.ArmL.xdot.jl.min = gain .* (pandaArm.jlmin + ((pandaArm.jlmin - pandaArm.ArmL.q) .* 0.1) - pandaArm.ArmL.q);
    pandaArm.ArmR.xdot.jl.min = gain .* (pandaArm.jlmin + ((pandaArm.jlmin - pandaArm.ArmR.q) .* 0.1) - pandaArm.ArmR.q);
    
    switch mission.phase
        case 1 
            % Reach the grasping point
            % LEFT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            [ang, lin] = CartError(pandaArm.ArmL.wTt, pandaArm.ArmL.wTg); % e.g. CartError(wTg, wTv) returns the error that makes <v> -> <g>
           
            pandaArm.ArmL.xdot.tool = gain * [ang; lin];
            % limit the requested velocities...
            pandaArm.ArmL.xdot.tool(1:3) = Saturate(pandaArm.ArmL.xdot.tool(1:3,:), 2);
            pandaArm.ArmL.xdot.tool(4:6) = Saturate(pandaArm.ArmL.xdot.tool(4:6,:), 2);
    
            % RIGHT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            [ang, lin] = CartError(pandaArm.ArmR.wTt, pandaArm.ArmR.wTg);
           
            pandaArm.ArmR.xdot.tool = gain * [ang; lin];
            % limit the requested velocities...
            pandaArm.ArmR.xdot.tool(1:3) = Saturate(pandaArm.ArmR.xdot.tool(1:3,:), 2);
            pandaArm.ArmR.xdot.tool(4:6) = Saturate(pandaArm.ArmR.xdot.tool(4:6,:), 2);
            
        % case 2 
        %     % Perform the rigid grasp of the object and move it
        % 
        %     % COMMON
        %     % -----------------------------------------------------------------
        %     % Rigid Grasp Constraint
        %     %pandaArm.xdot.rc = ...;
        % 
        %     % LEFT ARM
        %     % -----------------------------------------------------------------        
        %     % Object position and orientation task reference
        %     [ang, lin] = CartError();
        %     pandaArm.ArmL.xdot.tool = ...;
        %     % limit the requested velocities...
        %     pandaArm.ArmL.xdot.tool(1:3) = Saturate();
        %     pandaArm.ArmL.xdot.tool(4:6) = Saturate();
        % 
        %     % RIGHT ARM
        %     % -----------------------------------------------------------------
        %     % Object position and orientation task reference
        %     [ang, lin] = CartError();
        %     pandaArm.ArmR.xdot.tool = ...;
        %     % limit the requested velocities...
        %     %pandaArm.ArmR.xdot.tool(1:3) = Saturate();
        %     %pandaArm.ArmR.xdot.tool(4:6) = Saturate();
        % case 3
        %     % Stop any motions
        %     % LEFT ARM
        %     % -----------------------------------------------------------------
        %     % Tool position and orientation task reference
        %     %pandaArm.ArmL.xdot.tool(1:3) = ...;
        %     %pandaArm.ArmL.xdot.tool(4:6) = ...;
        % 
        %     % RIGHT ARM
        %     % -----------------------------------------------------------------
        %     % Tool position and orientation task reference
        %     %pandaArm.ArmR.xdot.tool(1:3) = ...;
        %     %pandaArm.ArmR.xdot.tool(4:6) = ...;
    end
end


