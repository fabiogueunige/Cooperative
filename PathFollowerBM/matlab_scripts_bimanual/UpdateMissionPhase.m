function [pandaArm, mission, goal] = UpdateMissionPhase(pandaArm, mission, goal)  
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
                    goal.counter = 1;

                    % update with the new goals
                    goal.previous = pandaArm.ArmL.wTo(:, :);
                    goal.future = goal.wTog(:, :, goal.counter);
                    goal.los = GetLosPoint(pandaArm.ArmL.wTo, goal.previous, goal.future); % compute the los
                 end   

            case 2 % Cooperative Manipulation Start 
                if(goal.counter < goal.n_goal) % case when I need to reach the target exept the last one
                    goal.los = GetLosPoint(pandaArm.ArmL.wTo, goal.previous, goal.future);
                   
                    % computing the errors for the rigid move-to task
                    [ang, lin] = CartError(goal.future, goal.los); % case when the loss point is near the target, == change point
                    if (abs(lin) <= 1/1000 & ang <= deg2rad(1))
                        % update all the needed variable
                        goal.previous = goal.wTog(:, :, goal.counter); % the counter indicate the last target
                        goal.counter = goal.counter + 1;
                        goal.future = goal.wTog(:, :, goal.counter); % indicate the next target pose
                    end
                    
                else    
                    goal.los = GetLosPoint(pandaArm.ArmL.wTo, goal.previous, goal.future);
                    
                    % computing the errors for the rigid move-to task
                    % check the error on last point 
                    [ang_left, lin_left] = CartError(goal.wTog(:, :, goal.n_goal), pandaArm.ArmL.wTo);
                    [ang_right, lin_right] = CartError(goal.wTog(:, :, goal.n_goal), pandaArm.ArmR.wTo);
        
                    % max error: 1/1000 cm and 3deg
                    if abs(lin_left) <= 1/10 & ang_left <= deg2rad(1) & abs(lin_right) <= 1/10 & ang_right <= deg2rad(1)
                       mission.phase = 3;
                       mission.phase_time = 0;
                       mission.prev_action = mission.actions.coop_manip.tasks;
                       mission.current_action = mission.actions.end_motion.tasks;
                    end
                end
            case 3 % Finish motion
                
        end
end

