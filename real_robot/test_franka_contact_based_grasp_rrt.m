%% TEST EXAMPLE CONTACT-BASED GRASP RRT %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

addpath('../grippers/franka_collision_check');

global vec_time_cone_comp;
global vec_time_cone_red;
global vec_time_force;
global ik_failures;
global force_failures;

% Disabling all warnings
warning('off','all');

%% Define main parameters

% Scenarios
% scenario_name = 'franka_book_on_table_horizontal.m';
% scenario_name = 'franka_book_on_table_vertical.m';
% scenario_name = 'franka_book_on_shelf.m';
% scenario_name = 'franka_book_on_table_cluttered.m';
% scenario_name = 'franka_book_on_table_vertical_cluttered.m';
% scenario_name = 'franka_cp_book_on_table_vertical.m';
% scenario_name = 'franka_cp_book_on_table_horizontal.m';
scenario_name = 'franka_cp_books_on_kallax.m';
% scenario_name = 'franka_cp_books_on_kallax_boxes.m';
% scenario_name = 'franka_cp_boxes_on_table_vertical_cluttered.m';

% Robot name
robot_name = 'franka_emika_panda';

% Define PFmC related constants
dt_max = 0.25;              	% dt for getting a new pose from velocity cone
start_moved = true;         % to start from a moved pose
n_expand = 20000;         	% max num. of iteration for tree expansion
tol = 1;                    % tolerance in norm between hom mats for stopping
edge_types = {'positioning', 'moving', 'release'};
edge_weights = [1, 1, 1];
p_release = 0.5;            % probability of implementing a release and not moving

% Define PFcC related constants
mu_h_val = 0.8; mu_e_val = 0.2;     % friction constants
f_min_h_ac = 0.5; f_max_h_ac = 5;  	% max and min hand force norms for actuatability
f_min_h_pf = 0; f_max_h_pf = 5;  	% max and min hand force norms for par. force closure
f_min_e = 0; f_max_e = 2;           % max and min env force norms
m_min_h = 0; m_max_h = 0.7;        	% max and min hand moment norms
kh = 1000; ke = 1000;              	% contact stiffness
we = 0.1*[0;-1;0;0;0;0]*9.81;      	% Attention here that we should be expressed obj frame

% Define optimization params
Delta = 0.00005;    % a small margin for extra robustness

% Putting these force related params in a single vector
force_params = {mu_h_val, mu_e_val, f_min_h_ac, f_max_h_ac, ...
    f_min_h_pf, f_max_h_pf, f_min_e, f_max_e, m_min_h, m_max_h, ...
    kh, ke, we, Delta};

%% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

%% Initialize tree with the starting node
G = initialize_tree(obj_ini, obj_fin, franka, env);

%% Expand the tree
t_start = tic;
[G_out, ind_sol, nearest, iters] = expand_tree_real_robot(G, env, obj_fin, n_expand, tol,...
    edge_types, edge_weights, p_release, force_params, dt_max);
disp('Time for expanding '); plan_time = toc(t_start);

G_final = G_out;

%% Implement direct twists for all nodes with only hand contacts
% tic
% [G_final, ind_sol, nearest] = build_all_direct_twists(G_out, env, ...
%     obj_fin, edge_types, edge_weights, ind_sol, nearest);
% toc

%% Saving the tree
save('Tree_Franka_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev')

%% Computing numbers and saving
mean_time_cone_comp = mean(vec_time_cone_comp);
mean_time_cone_red = mean(vec_time_cone_red);
mean_time_force = mean(vec_time_force);
std_time_cone_comp = std(vec_time_cone_comp);
std_time_cone_red = std(vec_time_cone_red);
std_time_force = std(vec_time_force);
expansions = height(G_final.Nodes);
save('fun_times.mat','plan_time','iters','expansions', ...
    'mean_time_cone_comp','mean_time_cone_red','mean_time_force');

disp('IK Failures '); disp(ik_failures);
disp('Force Failures '); disp(force_failures);
disp('Iterations '); disp(iters);
disp('Expansions '); disp(height(G_final.Nodes));
disp('Planning Time '); disp(plan_time);

%% Preliminary plots
% Draw the object of all the nodes
% figure_hand = draw_tree_real_robot(env,obj_fin,G_out,...
%     axis_range,azim,elev);

% If needed to override the plot params
% azim = -33.5;
% elev = 40;
% axis_range = [-5 5 -5 5 -1 6];

% Plot the output tree with labels
figure;
LWidths = 1*G_final.Edges.Weight/max(G_final.Edges.Weight);
plot(G_final,'EdgeLabel',G_final.Edges.Type,'LineWidth',LWidths)
 
% Get and draw random long paths
rand_ID = randsample(2:height(G_final.Nodes),1);
P_rand = shortestpath(G_final,1,nearest);
% P_rand = [1 23];
figure_hand2 = draw_path_real_robot(env,obj_ini,obj_fin,franka,G_final,P_rand,...
    axis_range,azim,elev);
