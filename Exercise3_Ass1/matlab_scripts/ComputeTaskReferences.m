function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% vehicle position task reference
w_vehicle_target_distance = uvms.wTgv(1:3,4) - uvms.wTv(1:3,4);

uvms.xdot.gv = -0.6 * w_vehicle_target_distance;
uvms.xdot.gv = Saturate(uvms.xdot.gv, 0.8);

% attitude control task reference
[lin, ang] = CartError(uvms.wTgv, uvms.wTv);
uvms.xdot.ac = - 0.6 * ang;
uvms.xdot.ac = Saturate(uvms.xdot.ac, 0.8);

% altitude control to 0
uvms.xdot.ac0 = - 0.6 * (0 - uvms.altitude);

% task reference for minimum altitude
uvms.xdot.ma = 0.2 * (3 - uvms.altitude);

% task reference for horizontal attitude
uvms.xdot.ha = -0.4 * (uvms.p(4:5) - [deg2rad(2); deg2rad(2)]);
