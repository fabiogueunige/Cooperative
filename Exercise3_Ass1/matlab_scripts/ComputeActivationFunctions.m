function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

% vehicle position task
uvms.A.gv = eye(3); % equality task
uvms.A.vh = 1; % equality task

% vehicle minimum altitude task
uvms.A.ma = DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.altitude);

% horizontal attitude task activation function
uvms.A.ha = zeros(2);
uvms.A.ha(1,1) = IncreasingBellShapedFunction(deg2rad(2), deg2rad(6), 0, 1, abs(uvms.p(4))); % threshold for |roll| at 4° +- 2°
uvms.A.ha(2,2) = IncreasingBellShapedFunction(deg2rad(2), deg2rad(6), 0, 1, abs(uvms.p(5))); % threshold for |pitch| at 4° +- 2°

% joint limits task attivation function
uvms.A.jl = zeros(7); % matrix with on diagonal all the sigmoids
for k = 1:7
    % set a matix with on diagonal sum of two sigmoid, one for the maximum
    % limit and one for the minimum joints limit
    uvms.A.jl(k,k) = DecreasingBellShapedFunction(uvms.jlmin(k), uvms.jlmin(k) + uvms.jlmin(k) * 0.1, 0, 1, uvms.q(k)) ...
                        + IncreasingBellShapedFunction(uvms.jlmax(k) - uvms.jlmax(k) * 0.1, uvms.jlmax(k) , 0,1, uvms.q(k));
end
% attitude control activation function
uvms.A.ac = eye(2);

% altitude control to 0
uvms.A.ac0 = 1;