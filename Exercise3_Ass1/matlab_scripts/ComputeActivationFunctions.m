function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

% vehicle position task
if mission.phase == 2
    uvms.A.gv = eye(2); % equality task
    uvms.A.va = DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.altitude);;
else
    uvms.A.gv = eye(3); % equality task
end

uvms.A.vh = 1; % equality task

% vehicle minimum altitude task
uvms.A.ma = DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.altitude);

% horizontal attitude task activation function
uvms.A.ha = zeros(2);
uvms.A.ha(1,1) = IncreasingBellShapedFunction(deg2rad(2), deg2rad(6), 0, 1, abs(uvms.p(4))); % threshold for |roll| at 4° +- 2°
uvms.A.ha(2,2) = IncreasingBellShapedFunction(deg2rad(2), deg2rad(6), 0, 1, abs(uvms.p(5))); % threshold for |pitch| at 4° +- 2°

% attitude control activation function
uvms.A.ac = eye(2);

% altitude control to 0
uvms.A.ac0 = 1;