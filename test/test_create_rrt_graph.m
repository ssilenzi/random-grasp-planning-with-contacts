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
% scenario_name = 'book_vertical_empty.m';
% scenario_name = 'book_on_table.m';
% scenario_name = 'book_on_table_vertical.m';
% scenario_name = 'book_on_box_corner.m';
% scenario_name = 'book_on_shelf_no_other_books.m';
% scenario_name = 'book_on_shelf.m';
% scenario_name = 'book_on_table_cluttered.m';

% Robot name
robot_name = 'hand_example';
link_dims = 1.2*ones(4,1);
is_und = true;

% Define PFmC related constants
dt_max = 1.0;              	% dt for getting a new pose from velocity cone
n_expand = 50;              % max num. of iteration for tree expansion
edge_types = {'positioning', 'moving', 'release'};
edge_weights = [1, 1, 1];

%% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, robot, axis_range, azim, elev] = ...
    build_scenario(scenario_name, robot_name, link_dims, is_und);

%% Initialize tree with the starting node
G = initialize_tree(obj_ini, obj_fin, robot, env);

%% Expand the tree as wanted
[G_out,nearest] = expand_wanted_tree(G, ...
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
rand_ID = randsample(2:height(G_final.Nodes),1);
P_rand = shortestpath(G_final,1,nearest);
% P_rand = [1 23];
figure_hand2 = draw_path(env,obj_fin,G_final,P_rand,...
    axis_range,azim,elev);
