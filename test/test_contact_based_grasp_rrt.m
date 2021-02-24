%% TEST EXAMPLE CONTACT-BASED GRASP RRT %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Disabling all warnings
warning('off','all');

%% Define main parameters

% If needed to override the plot params
azim = 45.7;
elev = 50;
axis_range = [-5 5 -5 5 -1 6];

% Scenarios
% scenario_name = 'book_vertical_empty.m';
% scenario_name = 'book_on_table.m';
% scenario_name = 'book_on_table_vertical.m';
% scenario_name = 'book_on_box_corner.m';
% scenario_name = 'book_on_shelf_no_other_books.m';
% scenario_name = 'book_on_shelf.m';
scenario_name = 'book_on_table_cluttered.m';

% Robot name
robot_name = 'hand_example';
link_dims = 1.2*ones(4,1);
is_und = false;

% Define PFmC related constants
dt_max = 0.7;                   % dt for getting a new pose from velocity cone
start_moved = true;         % to start from a moved pose
n_expand = 20000;         	% max num. of iteration for tree expansion
tol = 1;                    % tolerance in norm between hom mats for stopping
edge_types = {'positioning', 'moving', 'release'};
edge_weights = [1, 1, 1];
p_release = 0.3;            % probability of implementing a release and not moving

% Define PFcC related constants
mu_h_val = 0.7; mu_e_val = 0.2;     % friction constants
f_min_h_ac = 0.5; f_max_h_ac = 5;  	% max and min hand force norms for actuatability
f_min_h_pf = 0; f_max_h_pf = 5;  	% max and min hand force norms for par. force closure
f_min_e = 0; f_max_e = 1.5;           % max and min env force norms
m_min_h = 0; m_max_h = 0.7;           % max and min hand moment norms
kh = 1000; ke = 1000;              	% contact stiffness
we = 0.1*[0;-1;0;0;0;0]*9.81;      	% Attention here that we should be expressed obj frame

% Define optimization params
Delta = 0.00005;    % a small positive margin for aiding convergence of the
% optimization with fmincon; used in checking sigmas

% Putting these force related params in a single vector
force_params = {mu_h_val, mu_e_val, f_min_h_ac, f_max_h_ac, ...
    f_min_h_pf, f_max_h_pf, f_min_e, f_max_e, m_min_h, m_max_h, ...
    kh, ke, we, Delta};

%% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, robot, axis_range, azim, elev] = ...
    build_scenario(scenario_name, robot_name, link_dims, is_und, we);

%% Initialize tree with the starting node
G = initialize_tree(obj_ini, obj_fin, robot, env);

%% Expand the tree
tic
[G_out, ind_sol, nearest] = expand_tree2(G, env, obj_fin, n_expand, tol,...
    edge_types, edge_weights, p_release, force_params, dt_max);
disp('Time for expanding '); toc;

G_final = G_out;

%% Implement direct twists for all nodes with only hand contacts
% tic
% [G_final, ind_sol, nearest] = build_all_direct_twists(G_out, env, ...
%     obj_fin, edge_types, edge_weights, ind_sol, nearest);
% toc

%% Saving the tree
save('Tree_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev')

%% Preliminary plots
% Draw the object of all the nodes
% figure_hand = draw_tree(env,obj_fin,G_out,...
%     axis_range,azim,elev);

% Plot the output tree with labels
figure;
LWidths = 1*G_final.Edges.Weight/max(G_final.Edges.Weight);
plot(G_final,'EdgeLabel',G_final.Edges.Type,'LineWidth',LWidths)

% Get and draw random long paths
rand_ID = randsample(2:height(G_final.Nodes),1);
P_rand = shortestpath(G_final,1,nearest);
figure_hand2 = draw_path(env,obj_fin,G_final,P_rand,...
    axis_range,azim,elev);
