function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)    
        switch mission.phase
            case 1  % Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                % se l'errore è minore di max error allora mission.phase = 2
                [ang_left, lin_left] = CartError(pandaArm.ArmL.wTt, pandaArm.ArmL.wTg);
                % debug code
                mission.error.lin = [mission.error.lin; lin_left];
                mission.error.ang = [mission.error.ang; ang_left];
                [ang_right, lin_right] = CartError(pandaArm.ArmR.wTt, pandaArm.ArmR.wTg);
                % max error: 1/10 cm and 1deg
                if lin_left <= 1/1000 & ang_left <= deg2rad(1) & lin_right <= 1/1000 & ang_right <= deg2rad(1)
                % PROBLABLY WE NEED TO ADJUST THE GAIN to use the if above
                %if lin_left <= 1/10 & lin_right <= 1/10 % & ang_right <= deg2rad(15) & ang_left <= deg2rad(15)
                    mission.phase = 2;
                    mission.phase_time = 0;
                end

                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task
                
                [ang_left, lin_left] = CartError(pandaArm.wTog, pandaArm.ArmL.wTo);
                [ang_right, lin_right] = CartError(pandaArm.wTog, pandaArm.ArmR.wTo);
    
                % max error: 1 cm and 3deg
                if lin_left <= 1/100 & ang_left <= deg2rad(3) & lin_right <= 1/100 & ang_right <= deg2rad(3)
                   mission.phase = 3;
                end
               
            case 3 % Finish motion
                
        end
end

