function [robot_out, success] = ...
    move_robot_to_points_and_wrist(robot_in,Cp_glob,wrist_d)

% MOVE ROBOT TO POINTS MINIMUM DIFFERENCE - Moves the robot finger-tips
% to the specified points using its inverse kinematics function. Here we
% would like to keep all the joints as near as possible to the current...
% So, using minimum difference in compute_differential_inverse_kinematics_george

%   Inputs:
%   robot_in  	- robot object with all its properties and functions
%   Cp_glob     - target contact points (rows) in global frame
%   Outputs:
%   robot_out   - updated robot struction with new configuration

% Setting the tasks (target points and don't change much hand joints)
xd(:,:,1) = [eye(3) Cp_glob(1,1:3).'; [0 0 0 1]];
xd(:,:,2) = [eye(3) Cp_glob(2,1:3).'; [0 0 0 1]];
xd(:,:,3) = [eye(3) wrist_d; [0 0 0 1]];

% Here we just want unilateral constraints (as we already have the wrist)
if size(robot_in.q,1) ~= size(robot_in.sig_act,1)
    sig_open_d = robot_in.sig_act(7);  
else
    sig_open_d = robot_in.sig_act(7:8);  
end

ne = robot_in.compute_differential_inverse_kinematics_george(xd, sig_open_d);

robot_out = robot_in;

% If any of the hand joint is open or positioning error great, do not
% accept ik solution
joint_1_closed = robot_out.q(7) < 0;
joint_2_closed = robot_out.q(8) < 0;
good_error = ne < 0.01;
success = good_error && joint_1_closed && joint_2_closed;
% disp('ne '); disp(ne),

end

