function [pandaArm] = UpdateTransforms(pandaArm,mission)
% the function updates all the transformations

% Left arm transformations
pandaArm.bTe = getTransform(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');

% <e> to <w>
pandaArm.wTe = ...;

% Transformation matrix from <t> to <w>
pandaArm.wTt = ...;

% <o> to <w> : ASSUME <t> = <g> during entire cooperation phase
if (mission.phase == 2)
    pandaArm.wTo = ...;
end

