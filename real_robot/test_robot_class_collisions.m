%% TEST FOR THE ROBOT CLASS - Collisions %%


close all
clear
clc
run(fullfile('..', 'tools', 'resolve_paths.m'))

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
% run('franka_book_on_table_cluttered.m')
run('franka_book_on_table_vertical_cluttered.m')
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
% rob_h = franka.plot();
if ~success
    warning('Did not get good IK sol for starting!');
end

toc

tic

% Moving franka to contacts
[franka, success] = move_franka_to_points(franka,Cp_h0);
rob_h = franka.plot();
if ~success
    warning('Did not get good IK sol for grasp!');
    franka.q > franka.up_joint_lims
    franka.q < franka.lo_joint_lims
    franka.q
end

toc

tic

% Checking for collsions
[bool_self_col, self_coll_pair_id, world_coll_pair_id] = ...
    franka.build_check_collisions(box_object, environment);
if self_coll_pair_id
    warning('Did not get good pose for grasp because self collision!');
end
if world_coll_pair_id
    warning('Did not get good pose for grasp because world collision!');
end

toc

franka.highlight_collisions(self_coll_pair_id, world_coll_pair_id, gca);
franka.show_collision_boxes(box_object, environment);


