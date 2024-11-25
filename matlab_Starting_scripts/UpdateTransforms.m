function [uvms] = UpdateTransforms(uvms)
    % Updates all the transformations for the UVMS (underwater vehicle-manipulator system).
    
    % Homogeneous transformation matrix from the world frame <w> to the vehicle frame <v>.
    % - uvms.p contains the vehicle position (x, y, z) and orientation (roll, pitch, yaw).
    uvms.wTv = [rotation(uvms.p(4), uvms.p(5), uvms.p(6)) uvms.p(1:3); 0 0 0 1]; 
    
    % Inverse of the transformation: from <v> to <w>.
    uvms.vTw = inv(uvms.wTv); 
    
    % Transformation from <v> (vehicle) to <g> (goal/tool).
    uvms.vTg = uvms.vTw * uvms.wTg; 
    
    % Compute the manipulator's Jacobian, time derivative of the Jacobian, and end-effector pose.
    % - `JacobianMaris2` is a custom function using uvms.q (joint angles).
    [uvms.bJe, uvms.djdq, uvms.bTe] = JacobianMaris2(uvms.q); 
    
    % Transformation from <v> (vehicle) to <e> (end-effector).
    uvms.vTe = uvms.vTb * uvms.bTe; 
    
    % Transformation from <v> (vehicle) to <t> (tool).
    uvms.vTt = uvms.vTe * uvms.eTt;
    
    % Transformation from <w> (world) to <t> (tool).
    uvms.wTt = uvms.wTv * uvms.vTt;
end
