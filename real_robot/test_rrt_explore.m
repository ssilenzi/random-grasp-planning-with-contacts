%% TEST - EXPLORE ROBOT RRT %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Enabling all warnings
warning('on','all');

%% Define main parameters

scenario_name = 'franka_cp_books_on_kallax.m';
robot_name = 'franka_emika_panda';

% Saved experiment files
file_name = 'franka_cp_books_on_kallax_corrected_1_fail.mat';

% Load the file
% load(fullfile('videos and mats', file_name));
load(fullfile('videos and mats/new_nearest', file_name));

% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

% % Ad hoc axis, azim and elev and initial rob
% shelf vid submission
axis_range = [ -0.6 1.1 -0.4 0.6 -0.1 1.7 ]; 
azim = -142.15;
elev = 4.9615;
% G_final.Nodes(1,:).Robot{1}.q = [0, -1.2000, 0, -2.5000, 0, 1.8845, -1.0000, 0.0300, 0.0300].';
% G_final.Nodes(4,:).Robot{1}.q = [0, -1.2000, 0, -2.5000, 0, 1.8845, -1.0000, 0.0300, 0.0300].';
% % cluttered vid submission
% axis_range = [ -0.5 0.85 -0.55 0.55 0 0.85 ]; 
% azim = 118.3;
% elev = 16;
% sliding vid submission
% axis_range = [ -0.2 0.85 -0.55 0.85 0 0.95 ]; 
% azim = 118.3;
% elev = 16;

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
figure_hand2 = draw_path_real_robot(env,obj_ini,obj_fin,franka,G_final,P_rand,...
    axis_range,azim,elev);

% figure_hand2 = draw_path_for_paper(env,obj_ini,obj_fin,franka,G_final,P_rand(1:2),...
%     axis_range,azim,elev);

% Draw the tree and then the plan
% figure_hand = draw_tree_and_plan(env,obj_ini,obj_fin,franka,G_final,P_rand,...
%     axis_range,azim,elev);
