% ComputeTaskReferences
% Computes the task reference velocities based on the mission and UVMS state
function [uvms] = ComputeTaskReferences(uvms, mission)
    % Compute Cartesian error (angular and linear) for tool-frame control
    [ang, lin] = CartError(uvms.vTg, uvms.vTt); % Error between goal and tool
    uvms.xdot.t = 0.2 * [ang; lin]; % Scale error to desired velocity (0.2 m/s)
    
    % Saturate the velocity references to avoid exceeding limits
    uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2); % Angular velocity limit
    uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2); % Linear velocity limit
end
