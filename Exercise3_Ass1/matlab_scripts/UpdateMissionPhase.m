function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
        % computing the errors for the go-to action defining tasks
            % se l'errore è minore di max error allora mission.phase = 2
            [ang, lin] = CartError(uvms.wTv, uvms.wTgv);
            
            % max error: 1/10 cm and 1deg ??? da rivedere i valori
            %if lin_left <= 1/1000 & ang_left <= deg2rad(1) & lin_right <= 1/1000 & ang_right <= deg2rad(1)
            % PROBLABLY WE NEED TO ADJUST THE GAIN to use the if above
            if (norm(lin) <= 5) && (norm(ang) <= deg2rad(10))
                mission.phase = 2;
                mission.phase_time = 0;
            end
        case 2
            %
    end
end

