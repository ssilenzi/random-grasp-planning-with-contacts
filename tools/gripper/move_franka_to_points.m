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

% Here we just want unilateral constraints to avoid the hand to open
q_open_d = franka_in.q(8:9);  

ne = franka_in.compute_differential_inverse_kinematics(Xd, q_open_d);

franka_out = franka_in;

% If any of the hand joint is open or positioning error great, do not
% accept ik solution
joint_1_closed = franka_out.q(7) < 0;
joint_2_closed = franka_out.q(8) < 0;
good_error = ne < 0.01;
success = good_error && joint_1_closed && joint_2_closed;
% disp('ne '); disp(ne),

end

