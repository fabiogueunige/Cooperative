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

                    % TRAJECTORY FOLLOWER: Initialize trajectory tracking
                    goal.trajectory.time_in_phase = 0.0;
                    goal.trajectory.current_segment = 1;
                    goal.trajectory.completed = false;
                 end   

            case 2 % Cooperative Manipulation with TRAJECTORY FOLLOWER
                % Update trajectory time
                goal.trajectory.time_in_phase = mission.phase_time;
                
                % Check if trajectory is completed based on time
                if goal.trajectory.time_in_phase >= goal.trajectory.duration
                    % Trajectory completed, check if we reached final pose with sufficient accuracy
                    [ang_left, lin_left] = CartError(goal.trajectory.poses(:, :, goal.trajectory.n_waypoints), pandaArm.ArmL.wTo);
                    [ang_right, lin_right] = CartError(goal.trajectory.poses(:, :, goal.trajectory.n_waypoints), pandaArm.ArmR.wTo);
        
                    % max error: 1/10 cm and 1deg
                    if abs(lin_left) <= 1/10 & ang_left <= deg2rad(1) & abs(lin_right) <= 1/10 & ang_right <= deg2rad(1)
                       mission.phase = 3;
                       mission.phase_time = 0;
                       mission.prev_action = mission.actions.coop_manip.tasks;
                       mission.current_action = mission.actions.end_motion.tasks;
                       goal.trajectory.completed = true;
                    end
                else
                    % Update current segment index based on time
                    for i = 1:(goal.trajectory.n_waypoints - 1)
                        if goal.trajectory.time_in_phase >= goal.trajectory.times(i) && ...
                           goal.trajectory.time_in_phase < goal.trajectory.times(i+1)
                            goal.trajectory.current_segment = i;
                            break;
                        end
                    end
                end
                
            case 3 % Finish motion
                % Do nothing, mission completed
        end
end

