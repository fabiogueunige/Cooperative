function [uvms] = ComputeJacobians(uvms, mission)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%

% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.J.t_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.J.t_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
uvms.J.t = [uvms.J.t_a uvms.J.t_v];

% compute all the needed value for compute the jacobian when mission.phase == 2
if (mission.phase == 2) % in thisncase the heading jacobian is different, aligning task 
   d = uvms.rock_center - uvms.wTv(1:3, 4); % distance vector between rock and rover
   nd = d / norm(d); % normalized distance 
   n = cross(uvms.wTv(1:3, 1), nd); % axis of rotation
   n_t = n';
end

% MA veichle minimum altitude Jacobian
w_kw = [0 0 1]'; % z-axis on world frame projected on world frame
v_kw = uvms.vTw(1:3,1:3) * w_kw; % projection on vehicle frame

% vehicle minimum altitude Jacobian
uvms.J.ma = [zeros(1,7) v_kw' zeros(1,3)];

% HA vehicle horizonal attitude Jacobian
uvms.J.ha = [zeros(1,10) 1 0 0;
            zeros(1,10) 0 1 0];

% JL vehicle joint limits Jacobian
uvms.J.jl = [eye(7) zeros(7,6)];

% VH vehicle heading control Jacobian
uvms.J.vh = [zeros(1,10) 0 0 1];

% VP vehicle position Jacobian projected on <w>
uvms.J.vp = [zeros(3,7) uvms.wTv(1:3,1:3) zeros(3)];

if (mission.phase == 2)
    J = n_t * [zeros(3, 7), -1/(norm(d)^2) * skew(d), -eye(3)]; % one row
    uvms.J.va = [zeros(1, 7), J(8:9), zeros(1, 3), 1]; % one row
end

% AC attitude control Jacobian
uvms.J.ac = zeros(2,13);
uvms.J.ac(1,11) = 1;
uvms.J.ac(2,12) = 1;

end