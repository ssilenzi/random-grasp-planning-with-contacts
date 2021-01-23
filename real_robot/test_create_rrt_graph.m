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

% Robot name
robot_name = 'franka_emika_panda';

% Define PFmC related constants
dt_max = 0.1;              	% dt for getting a new pose from velocity cone
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
