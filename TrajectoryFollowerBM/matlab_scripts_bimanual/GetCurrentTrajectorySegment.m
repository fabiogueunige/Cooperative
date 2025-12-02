function [segment_idx, t_start, t_end, pose_start, pose_end] = GetCurrentTrajectorySegment(t_current, trajectory)
% Inputs:
%   t_current: current time
%   trajectory: structure containing waypoints and timing
%               .poses: 4x4xN array of homogeneous transformations
%               .times: 1xN array of absolute times for each waypoint
%               .n_waypoints: number of waypoints
%
% Outputs:
%   segment_idx: index of current segment
%   t_start: start time of current segment
%   t_end: end time of current segment
%   pose_start: homogeneous transformation at segment start
%   pose_end: homogeneous transformation at segment end

    segment_idx = 1;
    for i = 1:(trajectory.n_waypoints - 1)
        if t_current >= trajectory.times(i) && t_current <= trajectory.times(i+1)
            segment_idx = i;
            break;
        end
    end
    
    segment_idx = max(1, min(segment_idx, trajectory.n_waypoints - 1));
    
    % Extract segment information
    t_start = trajectory.times(segment_idx);
    t_end = trajectory.times(segment_idx + 1);
    pose_start = trajectory.poses(:, :, segment_idx);
    pose_end = trajectory.poses(:, :, segment_idx + 1);
    
end
