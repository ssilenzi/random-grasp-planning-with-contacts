%% TEST EXAMPLE CONTACT-BASED GRASP RRT %%

%% Define main parameters

% Define plot constants
axis_range = [-5 5 0 10 0 10];
azim = 45.7; % 50; 
elev = 45;

% Saved experiment files
file_name = 'Tree_book_on_shelf2.mat';

% Load the file
load(fullfile('mat_tree', file_name));

%% Preliminary plots
% Draw the robots and the object of all the nodes
% figure_hand = draw_tree(env,obj_fin,G_out,...
%     axis_range,azim,elev);

% Plot the output tree with labels
% figure;
% LWidths = 1*G_out.Edges.Weight/max(G_out.Edges.Weight);
% plot(G_out,'EdgeLabel',G_out.Edges.Type,'LineWidth',LWidths)

% Get and draw random long paths
rand_ID = randsample(2:height(G_out.Nodes),1);
P_rand = shortestpath(G_out,1,3094);
figure_hand2 = draw_path(env,obj_fin,G_out,P_rand,...
    axis_range,azim,elev);

%% Explore the tree to find a solution


%% Plot the solution