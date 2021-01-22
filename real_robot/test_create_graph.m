%% TEST CREATION CONTACT-BASED GRASP RRT %%
% FOR REAL ROBOT

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Disabling all warnings
warning('off','all');

%% Define main parameters

% Scenarios
% scenario_name = 'franka_cp_book_on_table_vertical.m';
% scenario_name = 'franka_cp_book_on_table_horizontal.m';
scenario_name = 'franka_cp_books_on_kallax.m';

% Robot name
robot_name = 'franka_emika_panda';

% Define PFmC related constants
dt_max = 0.1;              	% dt for getting a new pose from velocity cone
start_moved = true;         % to start from a moved pose
n_expand = 500;         	% max num. of iteration for tree expansion
tol = 1;                    % tolerance in norm between hom mats for stopping
edge_types = {'positioning', 'moving', 'release'};
edge_weights = [1, 1, 1];
p_release = 0.3;            % probability of implementing a release and not moving

% Define PFcC related constants
mu_h_val = 0.7; mu_e_val = 0.2;     % friction constants
f_min_h_ac = 0.5; f_max_h_ac = 5;  	% max and min hand force norms for actuatability
f_min_h_pf = 0; f_max_h_pf = 5;  	% max and min hand force norms for par. force closure
f_min_e = 0; f_max_e = 2;           % max and min env force norms
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
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

%% Initialize tree with the starting node
G = initialize_tree(obj_ini, obj_fin, franka, env);

%% Getting nodes one by one
[G_out, ind_sol, nearest] = create_franka_graph(G, env, obj_fin, n_expand, tol,...
    edge_types, edge_weights);
disp('Time for expanding '); toc;

G_final = G_out;