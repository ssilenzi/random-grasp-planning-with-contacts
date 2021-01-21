%% TEST - Publish environment to planning scene %%

%% Cleaning up and initializing
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Enabling all warnings
warning('on','all');

%% Define main parameters and run scenario

scenario_name = 'franka_book_on_shelf.m';
robot_name = 'franka_emika_panda';

% Build environment, object (initial and final)
[obj_ini, obj_fin, env, all_boxes, franka, axis_range, azim, elev] = ...
    build_scenario_real_robot(scenario_name, robot_name);

%% Starting ROS and creating needed clients

rosshutdown
rosinit('172.16.0.6');


%% Publishing to planning scene the environment
boxes_to_planning_scene(env)