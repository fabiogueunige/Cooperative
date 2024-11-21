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

% cut the jacobian to the seventh

% Left Arm base to ee Jacobian
pandaArm.ArmL.bJe = geometricJacobian(pandaArm.ArmL.franka, ...
    [pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
% Right Arm base to ee Jacobian
pandaArm.ArmR.bJe = geometricJacobian(pandaArm.ArmR.franka, ...
    [pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

% Top three rows are angular velocities, bottom three linear velocities
pandaArm.ArmL.wJt  = ...;
pandaArm.ArmR.wJt  = ...;

if (mission.phase == 2)
    pandaArm.ArmL.wJo = ...; 
    pandaArm.ArmR.wJo = ...;

% Common Jacobians
pandaArm.Jjl = ...;

end