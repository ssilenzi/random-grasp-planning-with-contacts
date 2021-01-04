%% TEST FOR THE ROBOT CLASS - SPAWNING, PLOT, ETC. - with IK %%


close all
clear
clc
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Some parameters
dt = 0.1;               % dt for getting a new pose from velocity cone

% Robot name
robot_name = 'franka_emika_panda';

% Loading and showing the robot
franka = load_gripper(robot_name);
fig_h = figure('Color',[1 1 1], 'pos',[0 0 800 800], ...
    'WindowState', 'maximized');
rob_h = franka.plot();

% Load the environment and the box (both initial and final poses)
% run('franka_book_on_table_vertical.m')
% run('franka_book_on_shelf.m')
run('franka_book_on_table_cluttered.m')
tot_h = plot_scenario(environment,box_object, ...
    target_position,axis_range,azim,elev);

% Get object position as row
Co0 = box_object.T(1:3,4).';

tic

% Get contacts with the environment and plot
[Cp_e0, Cn_e0] = get_contacts(environment, box_object, box_object.T);
plot_contacts(Cp_e0, Cn_e0, [0 1 0], 1);

% Get random points on object faces
[Cp_h0, Cn_h0] = get_random_contacts_on_box_partial(box_object, ...
    franka.n_contacts, Cp_e0, Cn_e0, true, 1);

% Get the hand in the starting config
[q0, success] = franka.get_starting_config(Cp_h0, Cn_h0, Co0, box_object);
rob_h = franka.plot();
if ~success
    warning('Did not get good IK sol for starting!');
end
franka.q > franka.up_joint_lims
franka.q < franka.lo_joint_lims
franka.q

toc
tic

% Moving franka to contacts
[franka, success] = move_franka_to_points(franka,Cp_h0);
rob_h = franka.plot();
if ~success
    warning('Did not get good IK sol for grasp!');
end
franka.q > franka.up_joint_lims
franka.q < franka.lo_joint_lims
franka.q

toc

%% Moving object ik
tic
% Getting the cone and plotting if necessary
Cone0 = pfc_analysis(Cp_e0, Cn_e0, 3);
toc
% Selecting a combination vec. and moving the object
ind = randsample(1:size(Cone0,2),1);
alpha0 = zeros(size(Cone0,2),1); alpha0(ind) = 1; % selecting a random generator
tic
[success, box_obj1, twist01, d_pose01] = get_pose_from_cone(Cone0, ...
    box_object, environment, dt, alpha0);
toc
if ~success
    error('Could not get a good pose inside Cone');
end

% Finding the moved contact points and normals and new robot config
Hom_d_pose01 = twistexp(d_pose01); % homogeneous trans. corresponding to obj twist
Cp_h1 = transform_points(Cp_h0, Hom_d_pose01);      % transforming points
Cn_h1 = transform_vectors(Cn_h0, Hom_d_pose01);     % transforming normals
plot_contacts(Cp_h1, Cn_h1, [1 0 1]);

% Wanted wrist transform
wrist0 = franka.get_forward_kinematics();
wrist0 = wrist0(1:4,4,3); % previous wrist position
wrist1 = Hom_d_pose01 * wrist0;
wrist1 = wrist1(1:3);

tic

% Moving the robot with the wanted wrist transform
[franka, success] = move_franka_to_points(franka,Cp_h1,wrist1);
rob_h = franka.plot();
if ~success
    warning('Did not get good IK sol for moving!');
end
franka.q > franka.up_joint_lims
franka.q < franka.lo_joint_lims
franka.q

toc

plot_box(box_obj1.l, box_obj1. w,box_obj1.h, box_obj1.T, [0 0 0], true);


