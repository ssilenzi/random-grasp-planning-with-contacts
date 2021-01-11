%% TEST EXAMPLE CONTACT-BASED GRASP RRT %%

%% Define main parameters

% Define plot constants
axis_range = [-5 5 0 10 0 10];
azim = 45.7; % 50; 
elev = 45;

% Saved experiment files
file_name = 'book_on_shelf2.mat';

% Load the file
load(fullfile('mat_tree', file_name));

%% Preliminary plots
% Draw the robots and the object of all the nodes
% figure_hand = draw_tree(env,obj_fin,G_out,...
%     axis_range,azim,elev);

% Plot the output tree with labels
% figure;
% LWidths = 1*G_final.Edges.Weight/max(G_final.Edges.Weight);
% plot(G_final,'EdgeLabel',G_final.Edges.Type,'LineWidth',LWidths)

% Get and draw random long paths
rand_ID = randsample(2:height(G_final.Nodes),1);
P_rand = shortestpath(G_final,1,655);
figure_hand2 = draw_path(env,obj_fin,G_final,P_rand,...
    axis_range,azim,elev);

% To draw a single node
ID_rand = 56;
figure_hand3 = draw_node(env,obj_fin,G_final,ID_rand,...
    axis_range,azim,elev);

%% Explore the tree to find a solution


%% Plot the solution