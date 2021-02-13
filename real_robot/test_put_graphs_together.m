%% To put together two existing graphs

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

% Build environment, object (initial and final)
% [obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
%     build_scenario_real_robot(scenario_name, robot_name);

% Saved experiment files
file_name1 = 'franka_cp_boxes_on_table_vertical_cluttered_good2.mat';
file_name2 = 'franka_cp_boxes_on_table_vertical_cluttered1.mat';

%% Loading graphs and appending
% Load the file 1
load(fullfile('videos and mats', file_name1));
G_up = G_final;

load(fullfile('videos and mats', file_name2));
G_lo = G_final;

[G_final] = append_graphs(G_up,G_lo);

save('Tree_Franka_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev')

% Plots
figure;
LWidths = 1*G_final.Edges.Weight/max(G_final.Edges.Weight);
plot(G_final,'EdgeLabel',G_final.Edges.Type,'LineWidth',LWidths)

P_rand = shortestpath(G_lo,1,G_lo.Nodes(end,:).ID);
figure_hand2 = draw_path_real_robot(env,G_lo.Nodes(1,:).Object{1}, ...
    obj_fin,G_lo.Nodes(1,:).Robot{1},G_lo,P_rand,...
    axis_range,azim,elev);

