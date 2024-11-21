function [A] = ActionTransition(taskname, previous, current, time)
% This function produce the sigmoid function to activate or deactivate tasks
% according to the current and previous phase of the mission.
% Takes as input the current task name ("T","RC","JL","MA"), the previous
% task, the current one and the mission time. If doubts, see the Matlab doc of
% ismember()

if (ismember(taskname, previous) && ismember(taskname, current))
    A = 1;
elseif (ismember(taskname, previous) == 0 && ismember(taskname, current))
    A = IncreasingBellShapedFunction(0, 1, 0, 1, time);
elseif (ismember(taskname, previous) && ismember(taskname, current) == 0)
    A = DecreasingBellShapedFunction(0, 1, 0, 1, time);
else
    A = 0;

end

