function [franka_out, success] = move_franka_to_points(franka_in,Cp_glob,wrist_d)

% MOVE FRANKA TO POINTS - Moves the franka gripper finger-tips
% to the specified points using its inverse kinematics function. Here we
% just need the franka to be inside joint limits?

%   Inputs:
%   robot_in  	- robot object with all its properties and functions
%   Cp_glob     - target contact points (rows) in global frame
%   wrist_d     - desided wrist position, if none, use the present one
%   Outputs:
%   robot_out   - updated robot struction with new configuration

if ~exist('wrist_d', 'var')
    wrist_d = franka_in.X(:,:,3); % wrist to keep it near
else
    wrist_d = [eye(3) wrist_d; [0 0 0 1]];
end

% Setting the tasks (target points and don't change much hand joints)
Xd(:,:,1) = [eye(3) Cp_glob(1,1:3).'; [0 0 0 1]];
Xd(:,:,2) = [eye(3) Cp_glob(2,1:3).'; [0 0 0 1]];
Xd(:,:,3) = wrist_d;

% Here we just want unil. constr. for finger joint limits
ne = franka_in.compute_differential_inverse_kinematics(Xd, true, 8);
franka_out = franka_in;

% If any of the gripper joint is open or positioning error great, do not
% accept ik solution
is_viol_gripper_joints = ...
    any(franka_out.q(8:9) < franka_out.lo_joint_lims(8:9)) || ...
    any(franka_out.q(8:9) > franka_out.up_joint_lims(8:9));
ne
good_error = ne < 0.01;
success = good_error && ~is_viol_gripper_joints;
% disp('ne '); disp(ne),

end

