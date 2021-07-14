%% TEST CREATE GRASP RRT %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Enabling all warnings
warning('on','all');

%% Define main parameters

% Scenarios
% scenario_name = 'franka_cp_book_on_table_vertical.m';
% scenario_name = 'franka_cp_book_on_table_horizontal.m';
scenario_name = 'franka_cp_books_on_kallax.m';
% scenario_name = 'franka_cp_books_on_kallax_boxes.m';
% scenario_name = 'franka_cp_boxes_on_table_vertical_cluttered.m';
% scenario_name = 'franka_cp_boxes_on_table_vertical.m';

% Robot name
robot_name = 'franka_emika_panda';

% Define PFmC related constants
dt_max = 0.06;              	% dt for getting a new pose from velocity cone
n_expand = 20;              % max num. of iteration for tree expansion
edge_types = {'positioning', 'moving', 'release'};
edge_weights = [1, 1, 1];

%% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

%% Initialize tree with the starting node
G = initialize_tree(obj_ini, obj_fin, franka, env);

%% Expand the tree as wanted
[G_out,nearest] = expand_wanted_tree_real_robot(G, ...
    env,obj_fin,n_expand,edge_types,edge_weights,dt_max);

G_final = G_out;

%% Saving the tree
save('Tree_Franka_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev')

%% Preliminary plots
% Draw the object of all the nodes
% figure_hand = draw_tree_real_robot(env,obj_fin,G_out,...
%     axis_range,azim,elev);

% If needed to override the plot params
% azim = -33.5;
% elev = 40;
% axis_range = [-5 5 -5 5 -1 6];

% Plot the output tree with labels
figure;
LWidths = 1*G_final.Edges.Weight/max(G_final.Edges.Weight);
plot(G_final,'EdgeLabel',G_final.Edges.Type,'LineWidth',LWidths)
 
% Get and draw random long paths
nearest = height(G_final.Nodes)-1;
rand_ID = randsample(2:height(G_final.Nodes),1);
P_rand = shortestpath(G_final,1,nearest);
% P_rand = [1 23];
figure_hand2 = draw_path_real_robot(env,obj_ini,obj_fin,franka,G_final,P_rand,...
    axis_range,azim,elev);
