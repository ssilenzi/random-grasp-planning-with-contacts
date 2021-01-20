%% TEST - SEND WHOLE TREE TO ROBOT ONE BY ONE %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Enabling all warnings
warning('on','all');

%% Define main parameters

scenario_name = 'franka_cp_book_on_table_horizontal.m';
robot_name = 'franka_emika_panda';

% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

% Saved experiment files
file_name = 'franka_cp_book_on_table_horizontal_bad.mat';

% Load the file
load(fullfile('videos and mats', file_name));

%% Get a random node and draw
% rand_ID = randsample(2:height(G_final.Nodes),1);
rand_ID = height(G_final.Nodes);
rand_ID = 67;
[P_rand, dist_path, edge_path] = shortestpath(G_final,1,rand_ID);
% P_rand = [1 23];
figure_hand2 = draw_path_real_robot(env,obj_ini,obj_fin,franka,G_final,P_rand,...
    axis_range,azim,elev);

%% Starting ROS and creating needed clients

rosshutdown
rosinit('172.16.0.6');

% Creating service clients
plan_client = ...
    rossvcclient('/panda_gripper_manipulation/manipulation_path_plan_service');
control_client = ...
    rossvcclient('/panda_gripper_manipulation/robot_control_service');
wait_client = ...
    rossvcclient('/panda_gripper_manipulation/robot_wait_service');
gripper_open_pub = ...
    rospublisher('/panda_controllers/gripper_control', 'std_msgs/Float64');

% Creating srv or msg which won't change
waitreq = rosmessage(wait_client);
dur_msg = rosmessage('std_msgs/Duration'); dur_msg.Data = rosduration(30.0);
waitreq.WaitDuration = dur_msg;

% Creating msgs
start_joint_msg = rosmessage('sensor_msgs/JointState'); % is empty here
gripper_open_msg = rosmessage('std_msgs/Float64');
gripper_open_msg.Data = 0.0; % for opening

%% In a loop, fill req to be sent (putting all adjacent movs together)

planreq_arr = [];

i = 2;
while (i <= length(P_rand)) 
    
%     disp('In first while');
    
    % Get the current node info
    [arm_pose_i,gripper_pos_i,type_i,same_face_i,sym_cont_i] = ...
        get_planning_info_node(G_final, P_rand(i), P_rand(i-1));
    
    % Creating and filling up request
    planreq = rosmessage(plan_client);
    planreq.Waypoints = arm_pose_i;
    planreq.FingerStates = gripper_pos_i;
    planreq.StartArmState = start_joint_msg; % empty for now
    planreq.Transition = type_i;
    planreq.SameFaceContacts = same_face_i;
    planreq.SymContacts = sym_cont_i;
    
    % Checking if type mov and the next are mov
    j = i;
    if strcmp(type_i, 'mov')     
        is_move = true;
        while (j < length(P_rand))
%             disp('In second while');
            % Get the current node info
            [arm_pose_j,gripper_pos_j,type_j,same_face_j,sym_cont_j] = ...
                get_planning_info_node(G_final, P_rand(j+1), P_rand(j));
            if strcmp(type_j, 'mov')
                planreq.Waypoints = [planreq.Waypoints; arm_pose_j];
                j = j+1;
            else
                break;
            end            
        end
    end
    
    % Pushing back to array of plan requests
    planreq_arr = [planreq_arr, copy(planreq)];
    planreq.Waypoints = [];
    
    i = j+1;
    
end

%% Sending to plan and control all messages in array one by one

for i = 1:length(planreq_arr)
    
    % Waiting for the robot to reach the previous goal
    waitresp = call(wait_client,waitreq,'Timeout',100);
    
    % Calling service planning
    planresp = call(plan_client,planreq_arr(i),'Timeout',100);
    
    % If good, calling robot control
    if planresp.Answer
        
        % Creating and filling up control message
        controlreq = rosmessage(control_client);
        controlreq.ComputedTrajectory = planresp.ComputedTrajectory;
        controlreq.ComputedGripperPosition = planresp.ComputedGripperPosition;
        controlreq.Transition = planresp.Transition;
        controlreq.SameFaceContacts = planresp.SameFaceContacts;
        controlreq.SymContacts = planresp.SymContacts;
              
        % Calling service control
        controlresp = call(control_client,controlreq,'Timeout',100);
        
    end
        
end

% Waiting for the robot to reach the previous goal
waitresp = call(wait_client,waitreq,'Timeout',100);

% Opening the gripper
send(gripper_open_pub,gripper_open_msg)
pause(2)

% Retracting the robot
success = retract_franka(plan_client, control_client, start_joint_msg);
waitresp = call(wait_client,waitreq,'Timeout',100);

% Shutting down
rosshutdown


% [actClient,goalMsg] = rosactionclient('/panda_arm/position_joint_trajectory_controller/follow_joint_trajectory');
% 
% waitForServer(actClient)
