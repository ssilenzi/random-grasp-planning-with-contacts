%% FOR TESTING ALL MAIN STUFF %%
% (hand func., par. form-closure, forces distr., constr. compliance)
% integrating also partially covered face contacts and collisions

%% Cleaning up and defining constants
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Define main constants
dt_max = 0.1;          	% max dt for getting a new pose from velocity cone

%% Building scenario, object and hand
% Loading the hand
robot_name = 'franka_emika_panda';
franka = load_gripper(robot_name);
fig_h = figure('Color',[1 1 1], 'pos',[0 0 800 800], ...
    'WindowState', 'maximized');
rob_h = franka.plot([], false, gca);

% Build the scenario and the box
% run('franka_book_vertical_empty.m')
run('franka_book_on_table_vertical.m')
% run('franka_book_on_shelf.m')
% run('franka_book_on_table_cluttered.m')
% run('franka_book_on_table_vertical_cluttered.m')
% run('franka_cp_book_on_table_vertical.m')
tot_h = plot_scenario(environment,box_object, ...
    target_position,axis_range,azim,elev);

% plot_forces([0 0 1.5], we.',[0 0.5 0.5],dm_to_m); % Plotting gravity force

%% Getting cone and sampling
% Get object position as row
Co0 = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp_e0, Cn_e0] = get_contacts(environment, box_object, box_object.T);
plot_contacts(Cp_e0, Cn_e0, [0 1 0], dm_to_m);

% Getting the cone and plotting if necessary
Cone0 = pfc_analysis(Cp_e0, Cn_e0, 3);

% Selecting a combination vec. and moving the object
% ind = randsample(1:size(Cone0,2),1);
alpha0 = zeros(size(Cone0,2),1); alpha0(1) = 1; % rand gen.

[success, box_obj1, twist01, d_pose01] = get_pose_from_cone(Cone0, ...
    box_object, environment, dt_max, alpha0);

if ~success
    error('Could not get a good pose inside Cone');
end

plot_box(box_obj1.l, box_obj1. w, box_obj1.h, box_obj1.T, [0 1 1], true);

% plot_free_cone(Cone,dt,box_obj,all_boxes,axis_range,az,el)
