function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
    % Compute distance between tools for plotting
    pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));

    %% Compute minimum altitude reference ALWAYS = gain ((min_alt + delta) - altitude)
    gain = 0.6; % our choice (constant)
    gain_jl = 0.3;
    gain_alt = 0.3;
    gain_tool = 0.4;
    gain_obj = 0.6;
    delta = 0.05;
    min_alt = 0.15; % guarda se vanno definiti fuori
    
    pandaArm.ArmL.xdot.alt = ((delta + min_alt) - pandaArm.ArmL.wTt(3,4)) * [0; 0; 0; 0; 0; gain_alt];% generate a positive velocity, according with x-axis, before minimum altitude task is inactive
    pandaArm.ArmR.xdot.alt = ((delta + min_alt) - pandaArm.ArmR.wTt(3,4)) * [0; 0; 0; 0; 0; gain_alt];
    
    %% Compute joint limits task reference ALWAYS
    % Create a velocity away from the limits => move to the middle between jlmax and jlmin

    % joint limits corresponding to the actual Panda by Franka arm configuration
    pandaArm.ArmL.xdot.jl (1:7, 1) = gain_jl .* (((pandaArm.jlmax - pandaArm.jlmin)/2) - pandaArm.ArmL.q);
    pandaArm.ArmR.xdot.jl (8:14, 1) = gain_jl .* (((pandaArm.jlmax - pandaArm.jlmin)/2) - pandaArm.ArmR.q);
    
    %% PROVA INTEGRATORE
    %persistent integrated_error_L;

    %if isempty(integrated_error_L)
    %integrated_error_L = zeros(6,1); % Inizializza l'errore integrato a zero
    %end

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
                       
            pandaArm.ArmL.xdot.tool = gain_tool * [ang; lin]; %+ pandaArm.ArmL.xdot.tool_I;

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

           
            
            % LEFT ARM
            % -----------------------------------------------------------------        
            % Object position and orientation task reference
            [ang, lin] = CartError(pandaArm.wTog, pandaArm.ArmL.wTo);
            pandaArm.ArmL.xdot.tool = gain_obj * [ang; lin];
            % limit the requested velocities...
            pandaArm.ArmL.xdot.tool(1:3) = Saturate(pandaArm.ArmL.xdot.tool(1:3,:), 2);
            pandaArm.ArmL.xdot.tool(4:6) = Saturate(pandaArm.ArmL.xdot.tool(4:6,:), 2);
    
            % RIGHT ARM
            % -----------------------------------------------------------------
            % Object position and orientation task reference
            [ang, lin] = CartError(pandaArm.wTog, pandaArm.ArmR.wTo);
            pandaArm.ArmR.xdot.tool = gain_obj * [ang; lin];
            % limit the requested velocities...
            pandaArm.ArmR.xdot.tool(1:3) = Saturate(pandaArm.ArmR.xdot.tool(1:3,:), 2);
            pandaArm.ArmR.xdot.tool(4:6) = Saturate(pandaArm.ArmR.xdot.tool(4:6,:), 2);
    
        case 3
             % Stop any motions
             % LEFT ARM
             % -----------------------------------------------------------------
             % Tool position and orientation task reference
            pandaArm.ArmL.xdot.tool(1:3) = zeros(3,1);
            pandaArm.ArmL.xdot.tool(4:6) = zeros(3,1);

            % RIGHT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            pandaArm.ArmR.xdot.tool(1:3) = zeros(3,1);
            pandaArm.ArmR.xdot.tool(4:6) = zeros(3,1);
    end
end


 