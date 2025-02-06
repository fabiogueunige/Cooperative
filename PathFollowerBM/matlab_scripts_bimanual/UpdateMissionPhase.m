function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)  
        switch mission.phase
            case 1  % Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                % se l'errore è minore di max error allora mission.phase = 2
                [ang_left, lin_left] = CartError(pandaArm.ArmL.wTg, pandaArm.ArmL.wTt);
                [ang_right, lin_right] = CartError(pandaArm.ArmR.wTg, pandaArm.ArmR.wTt);
                % max error: 1/10 cm and 1deg
                if lin_left <= 1/1000 & ang_left <= deg2rad(1) & lin_right <= 1/1000 & ang_right <= deg2rad(1)               
                    mission.phase = 2;
                    mission.phase_time = 0;
                    mission.prev_action = mission.actions.go_to.tasks;
                    mission.current_action = mission.actions.coop_manip.tasks;

                    % compute tTo and  
                    pandaArm.ArmL.tTo = inv(pandaArm.ArmL.wTt) * pandaArm.ArmL.wTo; 
                    pandaArm.ArmR.tTo = inv(pandaArm.ArmR.wTt) * pandaArm.ArmR.wTo;
                    pandaArm.ArmL.tDo = pandaArm.ArmL.wTt(1:3,4) - pandaArm.ArmL.wTo(1:3,4);
                    pandaArm.ArmR.tDo = pandaArm.ArmR.wTt(1:3,4) - pandaArm.ArmR.wTo(1:3,4);

                    % initialize the counter 
                    pandaArm.goal.counter = 1;

                    % update with the new goals
                    pandaArm.goal.previous = pandaArm.ArmL.wTo(:, :);
                    pandaArm.goal.future = pandaArm.goal.wTog(:, :, pandaArm.goal.counter);
                    pandaArm.goal.los = GetLosPoint(pandaArm.ArmL.wTo, pandaArm.goal.previous, pandaArm.goal.future); % compute the los
                 end   

            case 2 % Cooperative Manipulation Start 
                if(pandaArm.goal.counter < 2) % case when I need to reach the target exept the last one
                    pandaArm.goal.los = GetLosPoint(pandaArm.ArmL.wTo, pandaArm.goal.previous, pandaArm.goal.future);
                   
                    % computing the errors for the rigid move-to task
                    [ang, lin] = CartError(pandaArm.goal.future, pandaArm.goal.los) % case when the loss point is near the target, == change point
                    if (abs(lin) <= 1/1000 & ang <= deg2rad(1))
                        % update all the needed variable
                        pandaArm.goal.previous = pandaArm.goal.wTog(:, :, pandaArm.goal.counter); % the counter indicate the last target
                        pandaArm.goal.counter = pandaArm.goal.counter + 1;
                        pandaArm.goal.future = pandaArm.goal.wTog(:, :, pandaArm.goal.counter); % indicate the next target pose
                    end
                    
                else    
                    pandaArm.goal.los = pandaArm.goal.wTog(:, :, 2);
                    
                    % computing the errors for the rigid move-to task
                    % check the error on last point 
                    [ang_left, lin_left] = CartError(pandaArm.goal.wTog(:, :, 4), pandaArm.ArmL.wTo);
                    [ang_right, lin_right] = CartError(pandaArm.goal.wTog(:, :, 4), pandaArm.ArmR.wTo);
        
                    % max error: 1/1000 cm and 3deg
                    if abs(lin_left) <= 1/1000 & ang_left <= deg2rad(1) & abs(lin_right) <= 1/1000 & ang_right <= deg2rad(1)
                       mission.phase = 3;
                       mission.phase_time = 0;
                       mission.prev_action = mission.actions.coop_manip.tasks;
                       mission.current_action = mission.actions.end_motion.tasks;
                    end
                end
            case 3 % Finish motion
                
        end
end

