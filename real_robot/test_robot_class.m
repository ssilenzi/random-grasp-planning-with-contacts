%% TEST FOR THE ROBOT CLASS - SPAWNING, PLOT, ETC. - with boxes env. %%


close all
clear
clc
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Load the environment and the box (both initial and final poses)
% run('book_vertical_empty.m')
% run('book_on_shelf_no_target.m')
% run('book_on_shelf_no_other_books.m')
run('book_on_table.m')
% run('book_on_table_cluttered_no_target.m')
axis([-2, 6, -5, 15, 0, 15]) % Change the axis and view
axis equal
view([1, 1, 1])
zoom(1)
legend off

% Robot name
robot_name = 'franka_emika_panda';

% Loading and showing the robot
franka = load_gripper(robot_name);

% Get object position as row
Co0 = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp_e0, Cn_e0] = get_contacts(environment, box_object, box_object.T);
plot_contacts(Cp_e0, Cn_e0);

% Get random points on object faces
[Cp_h0, Cn_h0] = get_random_contacts_on_box_partial(box_object, ...
    franka.n_contacts, Cp_e0, Cn_e0, true);

% Get the hand in the starting config
q0 = franka.get_starting_config(Cp_h0, Cn_h0, Co0);
robot.set_config(q0);
ht = robot.plot();
