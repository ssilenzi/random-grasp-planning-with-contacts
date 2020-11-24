%% FOR TESTING ALL MAIN STUFF %%
% (hand func., par. form-closure, forces distr., constr. compliance)

%% Cleaning up and defining constants
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Define main constants
axis_range = [-10 10 -15 15 -15 15];
azim = 50; elev = 30;
dt = 0.5;               % dt for getting a new pose from velocity cone
num_hand_conts = 2;     % number of hand contacts
do_aux_plots = true;    % for plotting extra stuff

% Define force related constants
mu_hand_val = 0.7; mu_env_val = 0.2;    % friction constants
kh = 1000; ke = 1000;                   % contact stiffness

%% Building scenario, object and hand
% Build the scenario and the box (only initial pose)
% run('book_on_table.m')
% run('book_on_shelf_no_other_books.m')
% run('book_on_shelf_no_target.m')
run('book_on_table_cluttered_no_target.m')

axis(axis_range); axis equal; % Change the axis and view
view(azim, elev);
legend off;

% Loading the hand
robot = load_gripper('hand_example');

%% Getting cone and sampling
% Get object position as row
Co0 = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp_e0, Cn_e0] = get_contacts(environment, box_object, box_object.T);
plot_contacts(Cp_e0, Cn_e0);

% Getting the cone and plotting if necessary
Cone0 = pfc_analysis(Cp_e0, Cn_e0, 3);
% plot_free_cone(Cone,dt,box_object,all_boxes,axis_range,azim,elev);

% Selecting a combination vec. and moving the object
alpha0 = zeros(size(Cone0,2),1); alpha0(2) = 1; % selecting a generator
[box_obj1, twist1, d_pose1] = get_pose_from_cone(Cone0, box_object, dt, alpha0);
plot_box(box_obj1.l, box_obj1. w,box_obj1.h, box_obj1.T, [0 0 0], true);

%% Moving robot to random points and checking obj. motion compatibility
% Getting random contacts on free faces
[Cp_h0, Cn_h0] = get_random_contacts_on_box(box_object, num_hand_conts, ...
    Cp_e0, Cn_e0, do_aux_plots);

% Loading the hand in a starting pose
q0 = robot.get_starting_config_george(Cp_h0, Cn_h0);
robot.set_config(q0);
rob_handle0 = robot.plot();

% Moving robot to contacts
robot = move_robot_to_points(robot,Cp_h0);
rob_handle01 = robot.plot();

% Checking hand-kin obj-motion compatibility
if(~is_compatible_motion_hand_kin(robot,Cp_h0,Cn_h0,d_pose1))
    error('Cannot go on here, need to change hand contacts or object motion');
end

%% Analysis of contact point behaviour (getting contact types)
[Cp_e0_d, Cn_e0_d, cf_e_dim0, c_types0] = contact_type_analysis(Cp_e0, ...
    Cn_e0, d_pose1);







