%% FOR TESTING RANDOM CONTACT SELECTION %%

%% Cleaning up and defining constants
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Define main constants
axis_range = [-15 15 -15 15 -15 15];
azim = 50; elev = 30;
dt = 1.5;               % dt for getting a new pose from velocity cone
num_hand_conts = 2;     % number of hand contacts
do_aux_plots = true;    % for plotting extra stuff

%% Building scenario, object and hand
% Build the scenario and the box (only initial pose)
% run('book_on_table.m')
% run('book_on_table_vertical.m')
% run('book_on_box_corner_no_target.m')
% run('book_on_shelf_no_other_books.m')
run('book_on_shelf_no_target.m')
% run('book_on_table_cluttered_no_target.m')

axis(axis_range); axis equal; % Change the axis and view
view(azim, elev);
legend off;

%% Getting cone and sampling along a generator
% Get object position as row
Co0 = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp_e0, Cn_e0] = get_contacts(environment, box_object, box_object.T);
% plot_contacts(Cp_e0, Cn_e0);

% Getting random contacts on free parts
% [Cp_h0, Cn_h0] = get_random_contacts_on_box_partial(box_object, num_hand_conts, ...
%     Cp_e0, Cn_e0, do_aux_plots);

% % Getting the cone and plotting if necessary
Cone0 = pfc_analysis(Cp_e0, Cn_e0, 3);
% plot_free_cone(Cone,dt,box_object,all_boxes,axis_range,azim,elev);

% Selecting a combination vec. and moving the object
alpha0 = zeros(size(Cone0,2),1); alpha0(2) = 1; %alpha0(5) = 1; % selecting a generator
[box_obj1, twist01, d_pose01] = get_pose_from_cone(Cone0, box_object, dt, alpha0);
plot_box(box_obj1.l, box_obj1. w,box_obj1.h, box_obj1.T, [0 0 0], true);

%% Getting random points on the accessible parts of the new obj pose
% Get contacts with the environment and plot
[Cp_e1, Cn_e1] = get_contacts(environment, box_obj1, box_obj1.T);
plot_contacts(Cp_e1, Cn_e1);

% Getting random contacts on free parts
[Cp_h1, Cn_h1] = get_random_contacts_on_box_partial(box_obj1, num_hand_conts, ...
    Cp_e1, Cn_e1, do_aux_plots);


