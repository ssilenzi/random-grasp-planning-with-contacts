%% TEST EXAMPLE CONTACT-BASED GRASP RRT %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

%% Define main parameters

% Define plot constants
axis_range = [-15 15 -15 15 -15 15];
azim = 50; elev = 30;
do_aux_plots = true;    % for plotting extra stuff

% Scenarios
% scenario_name = 'book_on_table.m';
% scenario_name = 'book_on_table_vertical.m';
% scenario_name = 'book_on_box_corner.m';
% scenario_name = 'book_on_shelf_no_other_books.m';
% scenario_name = 'book_on_shelf.m';
scenario_name = 'book_on_table_cluttered.m';

% Robot name
robot_name = 'hand_example';

% Define PFmC related constants
dt = 1.2;               % dt for getting a new pose from velocity cone
num_hand_conts = 2;     % number of hand contacts
start_moved = true;  	% to start from a moved pose
n_expand = 50;         	% max num. of iteration for tree expansion
tol = 0.01;             % tolerance in norm between hom mats for stopping
edge_types = ['spawning', 'positioning', 'moving', 'release'];
edge_weights = [1, 1, 1, 1];
p_release = 0.1;       	% probability of implementing a release and not moving

% Define PFcC related constants
mu_h_val = 0.7; mu_e_val = 0.2;     % friction constants
f_min_h_ac = 0.5; f_max_h_ac = 5;  	% max and min hand force norms for actuatability
f_min_h_pf = 0; f_max_h_pf = 5;  	% max and min hand force norms for par. force closure
f_min_e = 0; f_max_e = 2;           % max and min hand force norms
kh = 1000; ke = 1000;              	% contact stiffness
we = 0.1*[0;-1;0;0;0;0]*9.81;      	% Attention here that we should be expressed obj frame

% Define optimization params
Delta = 0.00005;    % a small positive margin for aiding convergence of the
% optimization with fmincon; used in checking sigmas

%% Build environment, object (initial and final)
[obj_ini,obj_fin,env,all_boxes,robot] = build_scenario(scenario_name,...
    robot_name,we,axis_range,azim,elev);

%% Initialize tree with the starting node
G = initialize_tree(obj_ini, robot, env);

%% Expand the tree
[G_out, ind_sol] = expand_tree(G, env, n_expand, tol,...
    edge_types, edge_weights, p_release);

%% Explore the tree to find a solution


%% Plot the solution