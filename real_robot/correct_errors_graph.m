%% To solve errors in Graph labeling

close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Saved experiment files
file_name = 'franka_book_on_shelf2.mat';

% Load the file
load(fullfile('videos and mats', file_name));

% Solution
P_rand = shortestpath(G_final,1,G_final.Nodes(end,1).ID);

% Viewing the edges of solution
for i = 2:length(P_rand)
    ind_i = findedge(G_final,P_rand(i-1),P_rand(i));
    edge_i = G_final.Edges(ind_i,:);
    disp(edge_i(:,2).Type{1});
end

% For changing particular edges
for i = 6:length(P_rand)
    ind_i = findedge(G_final,P_rand(i-1),P_rand(i));
    G_final.Edges(ind_i,2).Type = {'moving'};
end
    

% %% Changing the last stuff
% for i = height(G_final.Edges):-1:height(G_final.Edges)-9
%     G_final.Edges(i,2).Type = {'moving'};
% end
% 
%% Saving the tree
save('Tree_Franka_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev')