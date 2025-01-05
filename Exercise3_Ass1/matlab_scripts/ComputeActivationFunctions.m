function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

% vehicle position task
uvms.A.gv = eye(3);

% vehicle minimum altitude task
uvms.A.ma = DecreasingBellShapedFunction(2, 3, 0, 1, uvms.altitude);