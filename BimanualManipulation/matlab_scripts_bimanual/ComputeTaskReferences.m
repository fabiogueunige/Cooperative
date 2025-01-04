function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
    % Compute distance between tools for plotting
    pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));

    %% Compute minimum altitude reference ALWAYS = gain ((min_alt + delta) - altitude)
    gain = 0.6; % our choice (constant)
    gain_jl = 0.3;
    gain_alt = 0.3;
    gain_tool = 0.3;
    gain_obj = 0.2;
    delta = 0.05;
    min_alt = 0.15; % guarda se vanno definiti fuori
    

    pandaArm.ArmL.xdot.alt = ((delta + min_alt) - pandaArm.ArmL.wTt(3,4)) * [0; 0; 0; 0; 0; gain_alt];% generate a positive velocity, according with x-axis, before minimum altitude task is inactive
    pandaArm.ArmR.xdot.alt = ((delta + min_alt) - pandaArm.ArmR.wTt(3,4)) * [0; 0; 0; 0; 0; gain_alt];
    
    %% Compute joint limits task reference ALWAYS
    % Create a velocity away from the limits => move to the middle between jlmax and jlmin

    % joint limits corresponding to the actual Panda by Franka arm configuration
    % (preso da init robot), controlla -> delta is 10% of jl - pos Joint

    pandaArm.ArmL.xdot.jl.max = gain_jl .* (pandaArm.jlmax + ((pandaArm.jlmax - pandaArm.ArmL.q) .* 0.1) - pandaArm.ArmL.q);
    pandaArm.ArmR.xdot.jl.max = gain_jl .* (pandaArm.jlmax + ((pandaArm.jlmax - pandaArm.ArmR.q) .* 0.1) - pandaArm.ArmR.q);
    
    pandaArm.ArmL.xdot.jl.min = gain_jl .* (pandaArm.jlmin + ((pandaArm.jlmin - pandaArm.ArmL.q) .* 0.1) - pandaArm.ArmL.q);
    pandaArm.ArmR.xdot.jl.min = gain_jl .* (pandaArm.jlmin + ((pandaArm.jlmin - pandaArm.ArmR.q) .* 0.1) - pandaArm.ArmR.q);

    %% PROVA INTEGRATORE
    persistent integrated_error_L;

    if isempty(integrated_error_L)
    integrated_error_L = zeros(6,1); % Inizializza l'errore integrato a zero
    end

    dt = 0.005; % Tempo di campionamento (modificare in base alla frequenza del ciclo)
    gain_I = 0.001; % Guadagno integrativo (modificare in base alle prestazioni desiderate)

    %%
    
    switch mission.phase
        case 1 
            % Reach the grasping point
            % LEFT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            [ang, lin] = CartError(pandaArm.ArmL.wTg, pandaArm.ArmL.wTt); % e.g. CartError(wTg, wTv) returns the error that makes <v> -> <g>
            
            % PROVA INTEGRATORE
            current_error_L = [ang; lin];  
            integrated_error_L = integrated_error_L + current_error_L * dt;

            pandaArm.ArmL.xdot.tool_I = gain_I * integrated_error_L;
           
            pandaArm.ArmL.xdot.tool = gain_tool * [ang; lin] + pandaArm.ArmL.xdot.tool_I;

            % limit the requested velocities...
            pandaArm.ArmL.xdot.tool(1:3) = Saturate(pandaArm.ArmL.xdot.tool(1:3,:), 2);
            pandaArm.ArmL.xdot.tool(4:6) = Saturate(pandaArm.ArmL.xdot.tool(4:6,:), 2);
    
            % RIGHT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            [ang, lin] = CartError(pandaArm.ArmR.wTg, pandaArm.ArmR.wTt);
           
            pandaArm.ArmR.xdot.tool = gain_tool * [ang; lin];
            % limit the requested velocities...
            pandaArm.ArmR.xdot.tool(1:3) = Saturate(pandaArm.ArmR.xdot.tool(1:3,:), 2);
            pandaArm.ArmR.xdot.tool(4:6) = Saturate(pandaArm.ArmR.xdot.tool(4:6,:), 2);
            
        case 2 
            % Perform the rigid grasp of the object and move it

            % COMMON
            % -----------------------------------------------------------------
            % Rigid Grasp Constraint
            % from theory:
            % [pandaArm.ArmL.wJo - pandaArm.ArmR.wJo] * ydotbar = 0
            % J * ydot = xdot --> xdot = 0
            pandaArm.xdot.rc = zeros(6,1);

            %pandaArm.xdot.rc = pandaArm.ArmL.wJo * pandaArm.ArmL.qdot + pandaArm.ArmR.wJo * pandaArm.ArmR.qdot; % Vo = J1*q1dot + J2*q2dot
            
            % LEFT ARM
            % -----------------------------------------------------------------        
            % Object position and orientation task reference
            [ang, lin] = CartError(pandaArm.wTog, pandaArm.ArmL.wTo);
            pandaArm.ArmL.xdot.obj = gain_obj * [ang; lin];
            % limit the requested velocities...
            pandaArm.ArmL.xdot.obj(1:3) = Saturate(pandaArm.ArmL.xdot.obj(1:3,:), 2);
            pandaArm.ArmL.xdot.obj(4:6) = Saturate(pandaArm.ArmL.xdot.obj(4:6,:), 2);
    
            % RIGHT ARM
            % -----------------------------------------------------------------
            % Object position and orientation task reference
            [ang, lin] = CartError(pandaArm.wTog, pandaArm.ArmR.wTo);
            pandaArm.ArmR.xdot.obj = gain_obj * [ang; lin];
            % limit the requested velocities...
            pandaArm.ArmR.xdot.obj(1:3) = Saturate(pandaArm.ArmR.xdot.obj(1:3,:), 2);
            pandaArm.ArmR.xdot.obj(4:6) = Saturate(pandaArm.ArmR.xdot.obj(4:6,:), 2);
    
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


 