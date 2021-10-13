%% TEST - MODIFY ROBOT AND WAYPOINTS %%

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

% Saved experiment files
file_name = 'franka_cp_book_on_table_horizontal1.mat';

% Load the file
load(fullfile('videos and mats', file_name));

% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

% Ad hoc axis, azim and elev and initial rob
% shelf
% axis_range = [ -0.6 1.1 -0.4 0.6 -0.1 1.1 ]; 
% azim = -142.15;
% elev = 4.9615;
% % cluttered
% axis_range = [ -0.2 0.85 -0.55 0.55 0 0.85 ]; 
% azim = 118.3;
% elev = 16;
% sliding
axis_range = [ -0.2 0.85 -0.65 0.95 0 0.85 ]; 
azim = 118.3;
elev = 16;

% Get solution
rand_ID = height(G_final.Nodes);
P_rand = shortestpath(G_final,1,rand_ID);

%% Move some obj poses and consequently robot (this need to be done ad hoc)
for i = 8:18 % for sliding
    
    i
    
    d_pose_direct = [0, (P_rand(i)/18)*0.15, 0, 0, 0, 0].'; 
    
    node_s = G_final.Nodes(P_rand(i),:);
    node_out = move_stuff_in_node_real_robot(node_s, ...
        d_pose_direct, env, axis_range, azim, elev);
    
%     pause;
    G_final.Nodes(P_rand(i),:) = node_out;
    
end

% Moving final obj and saving
obj_fin = twist_moves_object(obj_fin, [0, 0.15, 0, 0, 0, 0].');
save('Tree_Franka_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev');


%% Plots
% Draw solution
figure_hand2 = draw_path_real_robot(env,obj_ini,obj_fin,franka,G_final,P_rand,...
    axis_range,azim,elev);

