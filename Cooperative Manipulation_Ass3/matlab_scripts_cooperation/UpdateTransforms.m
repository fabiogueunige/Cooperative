function [pandaArm] = UpdateTransforms(pandaArm,mission)
% the function updates all the transformations

% Left arm transformations
pandaArm.bTe = getTransform(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');

% <e> to <w>
pandaArm.wTe = pandaArm.wTb * pandaArm.bTe;

% Transformation matrix from <t> to <w>
pandaArm.wTt = pandaArm.wTe * pandaArm.eTt;

% <o> to <w> : ASSUME <t> = <g> during entire cooperation phase
if (mission.phase == 2)
    % compute in phase 2 tTo 
    pandaArm.wTo = pandaArm.wTt * pandaArm.tTo;
end

