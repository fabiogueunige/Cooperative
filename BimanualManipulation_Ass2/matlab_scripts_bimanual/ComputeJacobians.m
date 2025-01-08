function [pandaArm] = ComputeJacobians(pandaArm,mission)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, 
% -0.0305686, 1.53975, 0.753872] ) 
%
% remember: the control vector is:
% [q_dot] 
% [qdot_1, qdot_2, ..., qdot_7]
%
% therefore all task jacobians should be of dimensions
% m x 14
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]

% cut the jacobian to the seventh column

% Left Arm base to ee Jacobian
pandaArm.ArmL.bJe = geometricJacobian(pandaArm.ArmL.franka, ...
    [pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
pandaArm.ArmL.bJe = pandaArm.ArmL.bJe(:,1:7); % reshape the bJe
% Right Arm base to ee Jacobian
pandaArm.ArmR.bJe = geometricJacobian(pandaArm.ArmR.franka, ...
    [pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT
pandaArm.ArmR.bJe = pandaArm.ArmR.bJe(:, 1:7); % reshape the bJe

% Rigid body jacobian through ee and tool
% Top three rows are angular velocities, bottom three linear velocities
pandaArm.ArmL.eSt = [eye(3) zeros(3,3); (skew(pandaArm.ArmL.wTe(1:3,1:3) * pandaArm.ArmL.eTt(1:3,4)))' eye(3)]; % rigid jacobian bethwwe ee and tool
pandaArm.ArmR.eSt = [eye(3) zeros(3,3); (skew(pandaArm.ArmR.wTe(1:3,1:3) * pandaArm.ArmR.eTt(1:3,4)))' eye(3)];

%jacobian from <w> to <t>: rigid-body_jabobian * wJe (wJe --> moltiplico bJe_linear per wRb e poi bJe_angular per wRb)
% wJt = eSt * bJe; 
pandaArm.ArmL.wJt = pandaArm.ArmL.eSt * [pandaArm.ArmL.wTb(1:3,1:3) zeros(3,3); zeros(3,3) pandaArm.ArmL.wTb(1:3,1:3)] * pandaArm.ArmL.bJe;
pandaArm.ArmR.wJt = pandaArm.ArmR.eSt * [pandaArm.ArmR.wTb(1:3,1:3) zeros(3,3); zeros(3,3) pandaArm.ArmR.wTb(1:3,1:3)] * pandaArm.ArmR.bJe;

if (mission.phase == 2)
    % tSo rigid jacobian between the robot and the obj 
    % needed distance fro tool to obj projected on world
    % 
    pandaArm.ArmL.tSo = [eye(3) zeros(3,3); (skew(pandaArm.ArmL.wTo(1:3,1:3) * (pandaArm.ArmL.tDo)))' eye(3)]; % rigid jacobian bethwwe ee and tool
    pandaArm.ArmR.tSo = [eye(3) zeros(3,3); (skew(pandaArm.ArmR.wTo(1:3,1:3) * (pandaArm.ArmR.tDo)))' eye(3)];
    %tTo calcolare in update mission phase andaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmL.wTo(1:3, 4))
    
    pandaArm.ArmL.wJo = pandaArm.ArmL.tSo * pandaArm.ArmL.wJt; 
    pandaArm.ArmR.wJo = pandaArm.ArmR.tSo * pandaArm.ArmR.wJt;

    pandaArm.Jrc = [pandaArm.ArmL.wJo, -pandaArm.ArmR.wJo];

end
%% Common Jacobians

%minimum altitude
pandaArm.ArmL.Jma(6,1:7) = pandaArm.ArmL.wJt(6,:); % row vector containing only the z linear components
pandaArm.ArmR.Jma(6,8:14) = pandaArm.ArmR.wJt(6,:);



end