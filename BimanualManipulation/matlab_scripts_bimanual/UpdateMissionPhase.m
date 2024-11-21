function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                
                % max error: 1/10 cm and 1deg
                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task

                % max error: 1 cm and 3deg
               
            case 3 % Finish motion
                
        end
end

