function [robot_out, success] = move_robot_to_points(robot_in,Cp_glob)
% MOVE ROBOT TO POINTS - Moves the robot end-effectors (hand finger-tips)
% to the specified points using its inverse kinematics function
%   Inputs:
%   robot_in  	- robot object with all its properties and functions
%   Cp_glob     - target contact points (rows) in global frame
%   Outputs:
%   robot_out   - updated robot struction with new configuration

% Getting the current wrist pose
x_now = robot_in.get_forward_kinematics();
x_wrist = x_now(:,:,3); % NOT USED AS OF NOW INSIDE IK

% Setting the tasks (target points and don't change much hand joints)
xd(:,:,1) = [eye(3) Cp_glob(1,1:3).'; [0 0 0 1]];
xd(:,:,2) = [eye(3) Cp_glob(2,1:3).'; [0 0 0 1]];
xd(:,:,3) = x_wrist;  % NOT USED AS OF NOW INSIDE IK
q_open_d = robot_in.q(7:8);

ne = robot_in.differential_inverse_kinematics(xd, q_open_d);

robot_out = robot_in;
% disp('ne '); disp(ne),
success = ne < 0.01;

end

