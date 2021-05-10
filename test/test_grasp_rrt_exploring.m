%% TEST EXAMPLE CONTACT-BASED GRASP RRT %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Enabling all warnings
warning('on','all');

%% Define main parameters
% Saved experiment files
file_name = 'book_on_table_cluttered_nearest_proj_cone1.mat';

% Load the file
% load(fullfile('new_mat_vids/', file_name));
% load(fullfile('mat_tree/old/', file_name));
% load(fullfile('mat_tree/2020-12-17/', file_name));
% load(fullfile('final_mats/', file_name));
load(fullfile('new_mat_vids/nearest_proj_cone', file_name));

% Define plot constants
% corner (its default)
% axis_range = [-8 5 -5 12 -1 6]; % table vert
axis_range = [-8 5 -5 8 -1 6]; % table hor
% axis_range = [-2 10 0 12 0 12]; % shelf
% axis_range = [-8 5 0 10 -5 10]; % cluttered
azim = 45.7; % 50 shelf ; 45.7 table and cluttered
elev = 50; % 30 shelf ; 50 table and cluttered

%% Preliminary plots

% Plot the output tree with labels
% figure;
% LWidths = 1*G_final.Edges.Weight/max(G_final.Edges.Weight);
% plot(G_final,'EdgeLabel',G_final.Edges.Type,'LineWidth',LWidths)

% Get and draw random long paths
fin_ID = height(G_final.Nodes);
% rand_ID = randsample(2:height(G_final.Nodes),1);
P_rand = shortestpath(G_final,1,fin_ID);
figure_hand2 = draw_path(env,obj_fin,G_final,P_rand,...
    axis_range,azim,elev);

% Draw the robots and the object of all the nodes
% figure_hand = draw_tree_and_plan(env,obj_fin,G_final,P_rand,...
%     axis_range,azim,elev);

% For saving if needed
% save('Tree_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev')

% To draw a single node
% ID_rand = 22;
% figure_hand3 = draw_node(env,obj_fin,G_final,ID_rand,...
%     axis_range,azim,elev);