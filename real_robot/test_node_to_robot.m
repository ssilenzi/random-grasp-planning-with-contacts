%% TEST - SEND NODE TO ROBOT %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Enabling all warnings
warning('on','all');

%% Define main parameters

scenario_name = 'franka_book_on_shelf.m';
robot_name = 'franka_emika_panda';

% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

% Saved experiment files
file_name = 'franka_book_on_shelf3.mat';

% Load the file
load(fullfile('videos and mats', file_name));

%% Get a random node and draw
% rand_ID = randsample(2:height(G_final.Nodes),1);
rand_ID = height(G_final.Nodes);
P_rand = shortestpath(G_final,1,rand_ID);
figure_hand2 = draw_path_real_robot(env,obj_ini,obj_fin,franka,G_final,P_rand,...
    axis_range,azim,elev);

pause;

%% Get node robot
node_2 = G_final.Nodes(P_rand(end-1),:); % row corresponding to P_rand
robot_2 = node_2.Robot{1};

node_3 = G_final.Nodes(P_rand(end),:); % row corresponding to P_rand
robot_3 = node_3.Robot{1};

% Getting needed data 2
hom_hand2 = robot_2.T_all(:,:,9);
pos_hand2 = hom_hand2(1:3,4);
rot_hand2 = hom_hand2(1:3,1:3);
quat_hand2 = rotm2quat(rot_hand2);
fing_pos2 = robot_2.q(end);

% Getting needed data 3
hom_hand3 = robot_3.T_all(:,:,9);
pos_hand3 = hom_hand3(1:3,4);
rot_hand3 = hom_hand3(1:3,1:3);
quat_hand3 = rotm2quat(rot_hand3);
fing_pos3 = robot_3.q(end);

%% Filling up message and sending to ros

% Starting ros
rosshutdown
rosinit('172.16.0.6');

% Creating service clients
plan_client = ...
    rossvcclient('/panda_gripper_manipulation/manipulation_path_plan_service');
control_client = ...
    rossvcclient('/panda_gripper_manipulation/robot_control_service');

% Creating and filling up srv
planreq = rosmessage(plan_client);
controlreq = rosmessage(control_client);

geom_msg2 = rosmessage('geometry_msgs/Pose');
geom_msg3 = rosmessage('geometry_msgs/Pose');
fing_joint_msg = rosmessage('sensor_msgs/JointState');
start_joint_msg = rosmessage('sensor_msgs/JointState'); % is empty here

geom_msg2.Position.X = pos_hand2(1);
geom_msg2.Position.Y = pos_hand2(2);
geom_msg2.Position.Z = pos_hand2(3)+0.2;
geom_msg2.Orientation.X = quat_hand2(2); % x
geom_msg2.Orientation.Y = quat_hand2(3); % y
geom_msg2.Orientation.Z = quat_hand2(4); % z
geom_msg2.Orientation.W = quat_hand2(1); % w

geom_msg3.Position.X = pos_hand3(1);
geom_msg3.Position.Y = pos_hand3(2);
geom_msg3.Position.Z = pos_hand3(3);
geom_msg3.Orientation.X = quat_hand3(2); % x
geom_msg3.Orientation.Y = quat_hand3(3); % y
geom_msg3.Orientation.Z = quat_hand3(4); % z
geom_msg3.Orientation.W = quat_hand3(1); % w

fing_joint_msg.Position = fing_pos2;

planreq.Waypoints = [geom_msg2];
planreq.FingerStates = fing_joint_msg;
planreq.StartArmState = start_joint_msg;

% Calling service planning
planresp = call(plan_client,planreq,'Timeout',20);

% If good, calling robot control
if planresp.Answer
    
    % Filling up control message
    controlreq.ComputedTrajectory = planresp.ComputedTrajectory;
    controlreq.ComputedGripperPosition = planresp.ComputedGripperPosition;
    
    % Calling service control
    controlresp = call(control_client,controlreq,'Timeout',30);
    
end

% Shutting down
rosshutdown


% [actClient,goalMsg] = rosactionclient('/panda_arm/position_joint_trajectory_controller/follow_joint_trajectory');
% 
% waitForServer(actClient)
