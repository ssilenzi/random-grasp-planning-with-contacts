%% FOR IMAGES FOR PAPER %%

%% Cleaning up and defining constants
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Define main constants
axis_range = [-15 15 -15 15 -15 15];
azim = 50; elev = 30;
dt = 3.0;               % dt for getting a new pose from velocity cone
num_hand_conts = 2;     % number of hand contacts
hand_cont_dim = 4;      % 3 if hard finger, 4 if soft finger
do_aux_plots = true;    % for plotting extra stuff
start_moved = false;  	% to start from a moved pose
n_try = 50;             % Max num. of tries for finding collision free stuff

%% Building scenario, object and hand
% Build the scenario and the box (only initial pose)
% run('book_vertical_empty.m')
% run('book_on_table.m')
% run('book_on_table_vertical.m')
% run('book_on_box_corner_no_target.m')
% run('book_on_shelf_no_other_books.m')
run('book_on_shelf_no_target.m')
% run('book_on_shelf.m')
% run('book_on_table_cluttered_no_target.m')

axis(axis_range); axis equal; % Change the axis and view
view(azim, elev);
legend off;

% Loading the hand
link_dims = 1.2*ones(4,1);
robot = load_gripper('hand_example', link_dims, true);

%% Getting cone and sampling
% Get object position as row
Co0 = box_object.T(1:3,4).';
tic
% Get contacts with the environment and plot
[Cp_e0, Cn_e0] = get_contacts(environment, box_object, box_object.T);
toc
tic
% Getting the cone and plotting if necessary
Cone0 = pfc_analysis(Cp_e0, Cn_e0, 3);
toc
% Selecting a combination vec. and moving the object
% ind = randsample([1:size(Cone0,2)],1);
alpha0 = zeros(size(Cone0,2),1); alpha0(1) = 1; % alpha0(ind) = 1;
tic
[success, box_obj1, twist01, d_pose01] = get_pose_from_cone(Cone0, ...
    box_object, environment, dt, alpha0);
toc
if ~success
    error('Could not get a good pose inside Cone');
end

% Plotting the initial stuff
% plot_contacts(Cp_e0, Cn_e0);
% plot_free_cone(Cone0,dt,box_object,all_boxes,axis_range,azim,elev);
plot_box(box_obj1.l, box_obj1. w,box_obj1.h, box_obj1.T, [0 0 0], true);

%% Moving robot to collision free random points and checking obj. motion compatibility
rob_coll = true;
tic
for i = 1:n_try
    
%     disp(i);
    % Getting random contacts on free faces
    [Cp_h0, Cn_h0] = get_random_contacts_on_box_partial(box_object, num_hand_conts, ...
        Cp_e0, Cn_e0, false);
    
    % Loading the hand in a starting pose
    sig0 = robot.get_starting_config_george(Cp_h0, Cn_h0, Co0);
    robot.set_act(sig0);
%     rob_handle0 = robot.plot();
    toc
    tic
    % Moving robot to contacts
    [robot, success] = move_robot_to_points(robot,Cp_h0);
    rob_handle01 = robot.plot();
    toc
    tic
    % Checking rob env collisions
    if ~success || robot.check_collisions({box_object}) ...
            || robot.check_collisions(environment)
        warning('Collision hand env detected');
%         delete(rob_handle0);
        delete(rob_handle01);
        % go further with the next random points
    else
        disp('Found a good hand pose');
        rob_coll = false;
        break; % the first ntry that is ok, is the way to go
    end
    toc
    
end
toc
plot_contacts(Cp_h0, Cn_h0, [0 1 0]);

if(rob_coll)
    error('Cannot go on here, did not find and rob env coll free pose');
end

%% Finding the moved contact points and normals and new robot config
Hom_d_pose01 = twistexp(d_pose01); % homogeneous trans. corresponding to obj twist
Cp_h1 = transform_points(Cp_h0, Hom_d_pose01);      % transforming points
Cn_h1 = transform_vectors(Cn_h0, Hom_d_pose01);     % transforming normals
plot_contacts(Cp_h1, Cn_h1, [0 1 0]);

% Wanted wrist transform
robot1 = copy(robot);
wrist0 = robot1.get_forward_kinematics();
wrist0 = wrist0(1:4,4,3); % previous wrist position
wrist1 = Hom_d_pose01 * wrist0;
wrist1 = wrist1(1:3);

% Moving the robot with the wanted wrist transform
[robot1, success] = move_robot_to_points_and_wrist(robot1,Cp_h1,wrist1);
if ~success
    warning('The hand was not moved correctly!');
end
rob_handle1 = robot.plot();

%% Moving the robot to a release configuration
Co1 = box_obj1.T(1:3,4).';
robot2 = copy(robot1);
sig2 = robot2.get_release_config_george(Cp_h1, Cn_h1, Co1);
robot2.set_act(sig2);
rob_handle2 = robot2.plot();

%% Plotting different figures
figure('units','normalized','outerposition',[0 0 1 1]);
axis equal;
axis off;
view(50, 30);
grid off
plot_boxes(environment, true, false);
plot_box(box_object.l, box_object.w, box_object.h, box_object.T, [0 0 1], true);

% Positioned
figure('units','normalized','outerposition',[0 0 1 1]);
axis equal;
axis off;
view(50, 30);
grid off
plot_boxes(environment, true, false);
plot_box(box_object.l, box_object.w, box_object.h, box_object.T, [0 0 1], true);
plot_contacts(Cp_h0, Cn_h0, [0 1 0]);
rob_handle = robot.plot();

% Moved
figure('units','normalized','outerposition',[0 0 1 1]);
axis equal;
axis off;
view(50, 30);
grid off
plot_boxes(environment, true, false);
plot_box(box_obj1.l, box_obj1.w, box_obj1.h, box_obj1.T, [0 0 1], true);
plot_contacts(Cp_h1, Cn_h1, [0 1 0]);
rob_handle = robot1.plot();

% Released
figure('units','normalized','outerposition',[0 0 1 1]);
axis equal;
axis off;
view(50, 30);
grid off
plot_boxes(environment, true, false);
plot_box(box_obj1.l, box_obj1.w, box_obj1.h, box_obj1.T, [0 0 1], true);
plot_contacts(Cp_h1, Cn_h1, [0 1 0]);
rob_handle = robot2.plot();

