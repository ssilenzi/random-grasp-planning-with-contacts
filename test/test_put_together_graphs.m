%% To put together two existing graphs - for simulated examples

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Enabling all warnings
warning('on','all');

% Define plot constants
% corner (its default)
% axis_range = [-8 5 -5 12 -1 6]; % table vert
% axis_range = [-8 5 -5 5 -1 6]; % table hor
axis_range = [-2 10 0 12 0 12]; % shelf
% axis_range = [-8 5 0 10 -5 10]; % cluttered
azim = 50; % 50 shelf ; 45.7 table and cluttered
elev = 30; % 30 shelf ; 50 table and cluttered

%% Define main parameters

% Saved experiment files
file_name1 = 'book_on_box_corner1.mat';
file_name2 = 'book_on_box_corner_paper1.mat';

%% Loading graphs and appending
% Load the file 1
% load(fullfile('mat_tree/2020-12-17/', file_name1));
load(fullfile('mat_tree/old/', file_name1));
G_up = G_final;

load(fullfile('new_mat_vids/', file_name2));
G_lo = G_final;

%% Appending and saving
[G_final] = append_graphs(G_up,G_lo);

save('Tree_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev')

%% Plots
% Plot tree
figure;
LWidths = 1*G_final.Edges.Weight/max(G_final.Edges.Weight);
plot(G_final,'EdgeLabel',G_final.Edges.Type,'LineWidth',LWidths)

% Get and draw random long paths
fin_ID = height(G_final.Nodes);
% rand_ID = randsample(2:height(G_final.Nodes),1);
P_rand = shortestpath(G_final,1,fin_ID);
figure_hand2 = draw_path(env,obj_fin,G_final,P_rand,...
    axis_range,azim,elev);