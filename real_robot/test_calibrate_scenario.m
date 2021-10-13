%% TEST CALIBRATION OF ROBOT ENVIRONMENT %%
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
% scenario_name = 'franka_cp_books_on_kallax.m';
scenario_name = 'franka_cp_books_on_kallax_boxes.m';

% Robot name
robot_name = 'franka_emika_panda';

% Define PFmC related constants
n_expand = 20;         	% max num. of iteration for tree expansion
edge_types = {'positioning', 'moving', 'release'};
edge_weights = [1, 1, 1];

%% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

%% Calibrate robot and scenario
calibrate_franka_scenario(franka);
