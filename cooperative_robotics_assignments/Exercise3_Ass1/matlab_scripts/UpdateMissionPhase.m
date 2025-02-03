function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  

        % computing the errors for the go-to action defining tasks
            % se l'errore è minore di max error allora mission.phase = 2
            [ang, lin] = CartError(uvms.wTv, uvms.wTgv);
            
            % max error: 1/10 cm and 1deg ??? da rivedere i valori
            %if lin_left <= 1/1000 & ang_left <= deg2rad(1) & lin_right <= 1/1000 & ang_right <= deg2rad(1)
            % PROBLABLY WE NEED TO ADJUST THE GAIN to use the if above
            if (lin <= 1/10) & (ang <= deg2rad(1))
                mission.phase = 2;
                mission.phase_time = 0;
                uvms.prev_action = uvms.act_action;
                uvms.act_action = uvms.actions.landing.tasks;
            end
        case 2
            if uvms.altitude <= 1/100 & uvms.angle <= deg2rad(5)
                mission.phase = 3;
                mission.phase_time = 0;
                uvms.prev_action = uvms.act_action;
                uvms.act_action = uvms.actions.fixed_base_manipulation.tasks;
            end
        case 3
            [ang, lin] = CartError(uvms.wTt, uvms.wTg);
            if (lin <= 1/100) & (ang <= deg2rad(1))
                mission.phase = 4;
                mission.phase_time = 0;
                uvms.prev_action = uvms.act_action;
            end

    end
end

