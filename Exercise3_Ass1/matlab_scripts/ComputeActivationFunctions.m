function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

% vehicle position task
uvms.A.gv = eye(3); % (equality task)
uvms.A.ac = uvms.A.gv;

% vehicle minimum altitude task
uvms.A.ma = DecreasingBellShapedFunction(2, 3, 0, 1, uvms.altitude);

% horizontal attitude task activation function
uvms.A.ha = zeros(2);
uvms.A.ha(1,1) = IncreasingBellShapedFunction(deg2rad(6), deg2rad(10), 0, 1, abs(uvms.p(4))); % threshold for |roll| at 8° +- 2°
uvms.A.ha(2,2) = IncreasingBellShapedFunction(deg2rad(6), deg2rad(10), 0, 1, abs(uvms.p(5))); % threshold for |pitch| at 8° +- 2°