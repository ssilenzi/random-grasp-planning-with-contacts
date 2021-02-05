%% TEST - EXPLORE ROBOT RRT %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Enabling all warnings
warning('on','all');

%% Define main parameters

scenario_name = 'franka_cp_books_on_kallax_for_paper.m';
robot_name = 'franka_emika_panda';

% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

% Saved experiment files
file_name = 'franka_cp_books_on_kallax2.mat';

% Load the file
load(fullfile('videos and mats', file_name));

% Ad hoc axis, azim and elev
axis_range = [ -0.2 1.1 -0.4 0.6 -0.1 1.1];
azim = -141.5762;
elev = 8.3369;

%% Preliminary plots
% Draw the robots and the object of all the nodes
% figure_hand = draw_tree(env,obj_fin,G_out,...
%     axis_range,azim,elev);

% Plot the output tree with labels
% figure;
% LWidths = 1*G_final.Edges.Weight/max(G_final.Edges.Weight);
% plot(G_final,'EdgeLabel',G_final.Edges.Type,'LineWidth',LWidths)

% Get and draw random long paths
% rand_ID = randsample(2:height(G_final.Nodes),1);
rand_ID = height(G_final.Nodes);
P_rand = shortestpath(G_final,1,rand_ID);
figure_hand2 = draw_path_real_robot(env,obj_ini,obj_fin,franka,G_final,P_rand(1:15),...
    axis_range,azim,elev);
