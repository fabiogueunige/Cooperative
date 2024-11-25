% ComputeActivationFunctions
% Computes activation matrices for each task based on mission priorities
function [uvms] = ComputeActivationFunctions(uvms, mission)
    % Tool-frame position control activation (always active)
    uvms.A.t = eye(6); % Full activation for tool-frame control
end
