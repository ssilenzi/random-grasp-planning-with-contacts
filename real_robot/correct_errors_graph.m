%% To solve errors in Graph labeling

close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Saved experiment files
file_name = 'franka_book_on_shelf3.mat';

% Load the file
load(fullfile('videos and mats', file_name));

%% Changing the last stuff
for i = height(G_final.Edges):-1:height(G_final.Edges)-9
    G_final.Edges(i,2).Type = {'moving'};
end

%% Saving the tree
save('Tree_Franka_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev')