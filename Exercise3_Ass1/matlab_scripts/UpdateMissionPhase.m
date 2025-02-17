function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  

        % computing the errors for the go-to action defining tasks
            % se l'errore è minore di max error allora mission.phase = 2
            [ang, lin] = CartError(uvms.wTv, uvms.wTgv);
    
            if (lin <= 1/10) %& (ang <= deg2rad(1))
                mission.phase = 2;
                mission.phase_time = 0;
                uvms.prev_action = uvms.act_action;
                uvms.act_action = uvms.actions.align.tasks;
            end
        case 2 % aligning task
            if abs(uvms.rho_w) <= deg2rad(1)
                mission.phase = 3;
                mission.phase_time = 0;
                uvms.prev_action = uvms.act_action;
                uvms.act_action = uvms.actions.landing.tasks;
            end
        case 3
            if uvms.altitude <= 1/100 
                mission.phase = 4;
                mission.phase_time = 0;
                uvms.prev_action = uvms.act_action;
                uvms.act_action = uvms.actions.fixed_base_manipulation.tasks;

            end
    end
end

