function [franka_out, success] = move_franka_to_points(franka_in,Cp_glob)

% MOVE FRANKA TO POINTS - Moves the franka gripper finger-tips
% to the specified points using its inverse kinematics function. Here we
% just need the franka to be inside joint limits?

%   Inputs:
%   robot_in  	- robot object with all its properties and functions
%   Cp_glob     - target contact points (rows) in global frame
%   Outputs:
%   robot_out   - updated robot struction with new configuration

% Setting the tasks (target points and don't change much hand joints)
Xd(:,:,1) = [eye(3) Cp_glob(1,1:3).'; [0 0 0 1]];
Xd(:,:,2) = [eye(3) Cp_glob(2,1:3).'; [0 0 0 1]];

% Here we just want unil. constr. for finger joint limits
ne = franka_in.compute_differential_inverse_kinematics(Xd, true, 8:9);
franka_out = franka_in;

% If any of the hand joint is open or positioning error great, do not
% accept ik solution
is_viol_joints = any(franka_out.q < franka_out.lo_joint_lims) || ...
    any(franka_out.q > franka_out.up_joint_lims);
ne
good_error = ne < 0.005;
success = good_error && ~is_viol_joints;
% disp('ne '); disp(ne),

end

