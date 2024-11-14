function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

w_vehicle_target_distance = uvms.wTgv(1:3,4) - uvms.wTv(1:3,4);
uvms.xdot.gv = -0.6 * w_vehicle_target_distance;
uvms.xdot.gv = Saturate(uvms.xdot.gv, 0.8);

uvms.xdot.ma = 0.2 * (3 - uvms.altitude);