function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)    
        switch mission.phase
            case 1  % Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                % se l'errore è minore di max error allora mission.phase = 2
                [ang_left, lin_left] = CartError(pandaArm.ArmL.wTt, pandaArm.ArmL.wTg);
                [ang_right, lin_right] = CartError(pandaArm.ArmR.wTt, pandaArm.ArmR.wTg);
                % max error: 1/10 cm and 1deg
                if lin_left <= 1/1000 && ang_left <= deg2rad(1) && lin_right <= 1/1000 && ang_right <= deg2rad(1)
                    mission.phase = 2;
                end

                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task

                % max error: 1 cm and 3deg
               
            case 3 % Finish motion
                
        end
end

