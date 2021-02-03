%% TEST EXAMPLE CONTACT-BASED GRASP RRT %%

%% Define main parameters

% Saved experiment files
file_name = 'Tree_book_on_shelf_good.mat';

% Load the file
load(fullfile('mat_tree/2020-12-17/', file_name));

% Define plot constants
axis_range = [-5 5 0 10 0 10];
azim = 50; % 50; 
elev = 30;

%% Preliminary plots
% Draw the robots and the object of all the nodes
% figure_hand = draw_tree(env,obj_fin,G_out,...
%     axis_range,azim,elev);

% Plot the output tree with labels
% figure;
% LWidths = 1*G_final.Edges.Weight/max(G_final.Edges.Weight);
% plot(G_final,'EdgeLabel',G_final.Edges.Type,'LineWidth',LWidths)

% Get and draw random long paths
% fin_ID = height(G_final.Nodes);
% % fin_ID = randsample(2:height(G_final.Nodes),1);
% P_rand = shortestpath(G_final,1,fin_ID);
% figure_hand2 = draw_path(env,obj_fin,G_final,P_rand,...
%     axis_range,azim,elev);

% To draw a single node
ID_rand = 22;
figure_hand3 = draw_node(env,obj_fin,G_final,ID_rand,...
    axis_range,azim,elev);