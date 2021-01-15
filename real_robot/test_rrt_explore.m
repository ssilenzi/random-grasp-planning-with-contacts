%% TEST - EXPLORE ROBOT RRT %%

%% Define main parameters

scenario_name = 'franka_book_on_shelf.m';
robot_name = 'franka_emika_panda';

% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

% Saved experiment files
file_name = 'franka_book_on_shelf3.mat';

% Load the file
load(fullfile('videos and mats', file_name));

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
