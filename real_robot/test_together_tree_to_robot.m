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
file_name = 'franka_cp_book_on_table_horizontal1.mat';

% Load the file
% load(fullfile('videos and mats/Old', file_name));
load(fullfile('videos and mats', file_name));

%% Get a random node and draw
% rand_ID = randsample(2:height(G_final.Nodes),1);
rand_ID = height(G_final.Nodes);
% rand_ID = randsample(2:height(G_final.Nodes),1);
% rand_ID = 207;
[P_rand, dist_path, edge_path] = shortestpath(G_final,1,rand_ID);
% P_rand = [1 23];
figure_hand2 = draw_path_real_robot(env,obj_ini,obj_fin,franka,G_final,P_rand,...
    axis_range,azim,elev);

%% Starting ROS and creating needed clients

rosshutdown
rosinit('172.16.0.6');

% Creating service clients, pubs and subs
plan_client = ...
    rossvcclient('/panda_gripper_manipulation/manipulation_path_plan_service');
control_client = ...
    rossvcclient('/panda_gripper_manipulation/robot_control_service');
wait_client = ...
    rossvcclient('/panda_gripper_manipulation/robot_wait_service');
gripper_open_pub = ...
    rospublisher('/panda_controllers/gripper_control', 'std_msgs/Float64');
joint_states_sub = rossubscriber('/joint_states');

% Creating srv or msg which won't change
waitreq = rosmessage(wait_client);
dur_msg = rosmessage('std_msgs/Duration'); dur_msg.Data = rosduration(30.0);
waitreq.WaitDuration = dur_msg;

% Creating msgs
start_joint_msg = rosmessage('sensor_msgs/JointState'); % is empty here
gripper_open_msg = rosmessage('std_msgs/Float64');
gripper_open_msg.Data = 0.0; % for opening

%% Publishing to planning scene the environment
% scale = 1;
% collision_boxes = {env{1}}; % Set here the needed boxes to planning scene
% boxes_to_planning_scene(collision_boxes, scale) 

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
    
%     % ADHOC For Cluttered
%     if P_rand(i) == 4
%         des_pose2 = [0.354, -0.021, 0.657, 0.950, -0.310, 0.035, -0.033]; % up
%         des_pose3 = [0.429, 0.210, 0.619, 0.879, 0.412, -0.064, 0.233]; % near
%         planreq2 = create_plan_req(plan_client, start_joint_msg, des_pose2, 'mov');
%         planreq_arr = [planreq_arr, copy(planreq2)];
%         planreq3 = create_plan_req(plan_client, start_joint_msg, des_pose3, 'mov');
%         planreq_arr = [planreq_arr, copy(planreq3)];
%     end
%     % End

    % ADHOC For Cluttered
    if P_rand(i) == 5
        des_pose2 = [0.298, 0.2904, 0.575, 0.262, 0.761, -0.228, 0.548]; % up
        des_pose3 = [0.327, 0.564, 0.239, 0.076, 0.724, -0.507, 0.462]; % near
        planreq2 = create_plan_req(plan_client, start_joint_msg, des_pose2, 'mov');
        planreq_arr = [planreq_arr, copy(planreq2)];
        planreq3 = create_plan_req(plan_client, start_joint_msg, des_pose3, 'mov');
        planreq_arr = [planreq_arr, copy(planreq3)];
    end
    % End
    
    i = j+1;
    
end

%% Sending to plan and control all messages in array one by one

% Getting stating joint states
joints_msg = receive(joint_states_sub, 5);

for i = 1:length(planreq_arr)
    
    % Waiting for the robot to reach the previous goal
    waitresp = call(wait_client,waitreq,'Timeout',100);
    
    % Calling service planning after setting start joints
    if i ~= 1 % setting prev. traj's last joints as start state
        traj_to_joint_states(joints_msg, ...
            controlreq.ComputedTrajectory.Points(end))
    end
    planreq_arr(i).StartArmState = joints_msg;
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
% success = retract_franka(plan_client, control_client, start_joint_msg, ...
%     [0, 0.05, 0], []); % shelf
% success = retract_franka(plan_client, control_client, start_joint_msg, ...
%     [-0.03, 0, 0], []); % cluttered
% waitresp = call(wait_client,waitreq,'Timeout',100);

% Bringing back to home
% home_pose = [0.38, 0.561, 0.512, 0.914, 0.147, 0.256, 0.279]; % shelf
% home_pose = [0.212, 0.240, 0.627, 0.362, 0.863, 0.090, 0.341]; % cluttered
home_pose = [0.348, -0.120, 0.699, 0.381, 0.912, 0.057, 0.145]; % sliding
success = retract_franka(plan_client, control_client, start_joint_msg, ...
    [], home_pose);
waitresp = call(wait_client,waitreq,'Timeout',100);

% Shutting down
rosshutdown


% [actClient,goalMsg] = rosactionclient('/panda_arm/position_joint_trajectory_controller/follow_joint_trajectory');
% 
% waitForServer(actClient)
