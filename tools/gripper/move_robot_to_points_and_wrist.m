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

% Here we just want minimum difference in ik
q_open_d = robot_in.q(7:8);  

ne = robot_in.compute_differential_inverse_kinematics_george(xd, q_open_d);

robot_out = robot_in;
% disp('ne '); disp(ne),
success = ne < 0.01;

end

