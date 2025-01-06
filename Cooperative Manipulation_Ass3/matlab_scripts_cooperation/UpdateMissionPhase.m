function [pandaArm, pandaArm2, mission] = UpdateMissionPhase(pandaArm, pandaArm2, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                [ang_left, lin_left] = CartError(pandaArm.wTt, pandaArm.wTg);
                [ang_right, lin_right] = CartError(pandaArm2.wTt, pandaArm2.wTg);

                mission.error.lin = [mission.error.lin; lin_left];
                mission.error.ang = [mission.error.ang; ang_left];
                
                % max error: 1/10 cm and 1deg 1/10*1/100
                if lin_left <= 1/1000 & ang_left <= deg2rad(1) & lin_right <= 1/1000 & ang_right <= deg2rad(1)
                % PROBLABLY WE NEED TO ADJUST THE GAIN to use the if above
                %if lin_left <= 1/10 & lin_right <= 1/10 % & ang_right <= deg2rad(15) & ang_left <= deg2rad(15)
                    mission.phase = 2;
                    mission.phase_time = 0;
                    % compute now tTo
                    pandaArm.tTo = inv(pandaArm.wTt) * pandaArm.wTo; 
                    pandaArm2.tTo = inv(pandaArm2.wTt) * pandaArm2.wTo;
                end

                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task

                % max error: 1 cm and 3deg
               
            case 3 % Finish motion
                
end

