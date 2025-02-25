function [pandaArm, pandaArm2, mission] = UpdateMissionPhase(pandaArm, pandaArm2, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
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
                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task
                [ang_left, lin_left] = CartError(pandaArm.wTog, pandaArm.wTo);
                [ang_right, lin_right] = CartError(pandaArm2.wTog, pandaArm2.wTo);
                % max error: 1 cm and 3deg
                if lin_left <= 1/1000 & ang_left <= deg2rad(1) & lin_right <= 1/1000 & ang_right <= deg2rad(1)
                    mission.phase = 3;
                    pandaArm.t3 = mission.phase_time; % storing the time at which we change phase (plot)
                    mission.phase_time = 0;
                    mission.prev_action = mission.current_action;
                    mission.act_action = mission.actions.end_motion.tasks;
                end
            case 3 % Finish motion
                
end

