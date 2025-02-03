function [pandaArm, pandaArm2, mission] = UpdateMissionPhase(pandaArm, pandaArm2, mission)    

        if mission.phase == 1  %Go To Grasping Points
            % computing the errors for the go-to action defining tasks
            [ang_left, lin_left] = CartError(pandaArm.wTg, pandaArm.wTt);
            [ang_right, lin_right] = CartError(pandaArm2.wTg, pandaArm2.wTt);

            mission.error.lin = [mission.error.lin; lin_left];
            mission.error.ang = [mission.error.ang; ang_left];
            
            % max error: 1/10 cm and 1deg 
            if lin_left <= 1/1000 & ang_left <= deg2rad(1) & lin_right <= 1/1000 & ang_right <= deg2rad(1)
                mission.phase = 2;
                pandaArm.t2 = mission.phase_time; % store the time at which we change phase (plot)
                mission.phase_time = 0;

                % tTo compute at grasping point (to avoid numerical errors)
                pandaArm.tTo = inv(pandaArm.wTt) * pandaArm.wTo; 
                pandaArm2.tTo = inv(pandaArm2.wTt) * pandaArm2.wTo;
                % distance from tool to object ompute at grasping point (to avoid numerical errors)
                pandaArm.tDo = pandaArm.wTt(1:3,4) - pandaArm.wTo(1:3,4);
                pandaArm2.tDo = pandaArm2.wTt(1:3,4) - pandaArm2.wTo(1:3,4);
                mission.prev_action = mission.current_action;
                mission.act_action = mission.actions.coop_manip.tasks;
             
            end
        end
        if (mission.phase == 2)% Cooperative Manipulation Start
            % computing the errors for the rigid move-to task
            [ang_left, lin_left] = CartError(pandaArm.wTog, pandaArm.wTo);
            [ang_right, lin_right] = CartError(pandaArm2.wTog, pandaArm2.wTo);
            % max error: 1 cm and 3deg
            if norm(lin_left) <= 1/100 && norm(ang_left) <= deg2rad(2) && norm(lin_right) <= 1/100 && norm(ang_right) <= deg2rad(2)
                mission.phase = 3;
                w_obj_g = [0.60; 0.40; 0.20]; % new goal for phase 3
                pandaArm.wTog(1:3,4) = w_obj_g;
                pandaArm2.wTog(1:3,4) = w_obj_g;
                pandaArm.t3 = mission.phase_time; % storing the time at which we change phase (plot)
                mission.phase_time = 0;
                mission.prev_action = mission.current_action;
                mission.act_action = mission.actions.end_motion.tasks;
            end
        end
        if mission.phase == 3
            [ang_left, lin_left] = CartError(pandaArm.wTog, pandaArm.wTo);
            [ang_right, lin_right] = CartError(pandaArm2.wTog, pandaArm2.wTo);
            %disp(pandaArm2.wTog(1:3,4))
            %disp(pandaArm.wTog(1:3,4))
            % max error: 1 cm and 3deg
            if norm(lin_left) <= 1/1000 && norm(ang_left) <= deg2rad(2) && norm(lin_right) <= 1/1000 && norm(ang_right) <= deg2rad(2)
                mission.phase = 4;
                w_obj_g = [0.60; 0.40; 0.58];
                pandaArm.wTog(1:3,4) = w_obj_g;
                pandaArm2.wTog(1:3,4) = w_obj_g;
                pandaArm.t3 = mission.phase_time; % storing the time at which we change phase (plot)
                mission.phase_time = 0;
                mission.prev_action = mission.current_action;
                mission.act_action = mission.actions.end_motion.tasks;
            end
        end
        if mission.phase == 4 % Finish motion
            [ang_left, lin_left] = CartError(pandaArm.wTog, pandaArm.wTo);
            [ang_right, lin_right] = CartError(pandaArm2.wTog, pandaArm2.wTo);
            % max error: 1 cm and 3deg
            if norm(lin_left) <= 1/1000 && norm(ang_left) <= deg2rad(2) && norm(lin_right) <= 1/100 && norm(ang_right) <= deg2rad(2)
                mission.phase = 5;
                w_obj_g = [0.60; 0.40; 0.20];
                pandaArm.wTog(1:3,4) = w_obj_g;
                pandaArm2.wTog(1:3,4) = w_obj_g;
                pandaArm.t3 = mission.phase_time; % storing the time at which we change phase (plot)
                mission.phase_time = 0;
                mission.prev_action = mission.current_action;
                mission.act_action = mission.actions.end_motion.tasks;
            end
                
        end
end

