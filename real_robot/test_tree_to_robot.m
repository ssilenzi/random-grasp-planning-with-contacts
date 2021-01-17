%% TEST - SEND WHOLE TREE TO ROBOT ONE BY ONE %%

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
[P_rand, dist_path, edge_path] = shortestpath(G_final,1,rand_ID);
figure_hand2 = draw_path_real_robot(env,obj_ini,obj_fin,franka,G_final,P_rand,...
    axis_range,azim,elev);

pause;

%% Starting ROS and creating needed clients

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

%% In a loop, get a node, plan (using also edge), then control
for i = 2:length(P_rand)
    
    % Get node info
    node_i = G_final.Nodes(P_rand(i),:); % row corresponding to P_rand
    robot_i = node_i.Robot{1};
    cont_h_i = node_i.Cn_h{1};
    
    % Checking if same face contact
    same_face_i = false;
    if ~isempty(cont_h_i)
        if (cont_h_i(1,:) == -cont_h_i(2,:))
            same_face_i = true;
        end
    end
    
    % Getting needed pose data
    hom_hand_i = robot_i.T_all(:,:,9);
    pos_hand_i = hom_hand_i(1:3,4);
    rot_hand_i = hom_hand_i(1:3,1:3);
    quat_hand_i = rotm2quat(rot_hand_i);
    fing_pos_i = robot_i.q(end);
    
    % Get previous edge type info
    ind = findedge(G_final, P_rand(i-1), P_rand(i));
    type_i = G_final.Edges(ind,2).Type{1};
    
    % Creating msgs
    geom_msg = rosmessage('geometry_msgs/Pose');
    fing_joint_msg = rosmessage('sensor_msgs/JointState');
    start_joint_msg = rosmessage('sensor_msgs/JointState'); % is empty here
    
    % Filling up msgs
    geom_msg.Position.X = pos_hand_i(1);
    geom_msg.Position.Y = pos_hand_i(2);
    geom_msg.Position.Z = pos_hand_i(3);
    geom_msg.Orientation.X = quat_hand_i(2); % x
    geom_msg.Orientation.Y = quat_hand_i(3); % y
    geom_msg.Orientation.Z = quat_hand_i(4); % z
    geom_msg.Orientation.W = quat_hand_i(1); % w    
    fing_joint_msg.Position = fing_pos2;
    
    % Filling up request
    planreq.Waypoints = geom_msg;
    planreq.FingerStates = fing_joint_msg;
    planreq.StartArmState = start_joint_msg; % empty for now
    planreq.Transition = type_i;
    planreq.SameFaceContacts = same_face_i;
    
    % Calling service planning
    planresp = call(plan_client,planreq,'Timeout',20);
    
    % If good, calling robot control
    if planresp.Answer
        
        % Filling up control message
        controlreq.ComputedTrajectory = planresp.ComputedTrajectory;
        controlreq.ComputedGripperPosition = planresp.ComputedGripperPosition;
        controlreq.Transition = planresp.Transition;
        controlreq.SameFaceContacts = planresp.SameFaceContacts;
        
        % Calling service control
        controlresp = call(control_client,controlreq,'Timeout',30);
        
    end
    
    
end

% Shutting down
rosshutdown


% [actClient,goalMsg] = rosactionclient('/panda_arm/position_joint_trajectory_controller/follow_joint_trajectory');
% 
% waitForServer(actClient)
