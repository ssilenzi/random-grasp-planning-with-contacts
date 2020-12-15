%% FOR TESTING DIRECT TWIST %%

%% Cleaning up and defining constants
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Define main constants
axis_range = [-15 15 -15 15 -15 15];
azim = 50; elev = 30;
dt = 1.0;               % dt for getting a new pose from velocity cone
num_hand_conts = 2;     % number of hand contacts
hand_cont_dim = 4;      % 3 if hard finger, 4 if soft finger
do_aux_plots = true;    % for plotting extra stuff
start_moved = false;  	% to start from a moved pose
n_try = 50;             % Max num. of tries for finding collision free stuff

% Graph stuff
edge_types = {'positioning', 'moving', 'release'};
edge_weights = [1, 1, 1];

%% Building scenario, object and hand
% Build the scenario and the box (only initial pose)
% run('book_vertical_empty.m')
% run('book_on_table.m')
run('book_on_table_vertical.m')
% run('book_on_box_corner.m')
% run('book_on_shelf_no_other_books.m')
% run('book_on_shelf.m')
% run('book_on_table_cluttered_.m')

axis(axis_range); axis equal; % Change the axis and view
view(azim, elev);
legend off;

% Loading the hand
link_dims = 1.2*ones(4,1);
robot = load_gripper('hand_example', link_dims);

%% Getting needed info
% Get object position as row
Co0 = box_object.T(1:3,4).';
tic
% Get contacts with the environment and plot
[Cp_e0, Cn_e0] = get_contacts(environment, box_object, box_object.T);
toc
tic
% Getting the cone and plotting if necessary
Cone0 = pfc_analysis(Cp_e0, Cn_e0, 3);

%% Moving robot to collision free random points
rob_coll = true;
for i = 1:n_try
    
%     disp(i);
    % Getting random contacts on free faces
    [Cp_h0, Cn_h0] = get_random_contacts_on_box_partial(box_object, num_hand_conts, ...
        Cp_e0, Cn_e0, true);
    
    % Loading the hand in a starting pose
    q0 = robot.get_starting_config_george(Cp_h0, Cn_h0, Co0);
    robot.set_config(q0);
    rob_handle0 = robot.plot();
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
        delete(rob_handle0);
        delete(rob_handle01);
        % go further with the next random points
    else
        disp('Found a good hand pose');
        rob_coll = false;
        break; % the first ntry that is ok, is the way to go
    end
    toc
    
end

if(rob_coll)
    error('Cannot go on here, did not find and rob env coll free pose');
end

%% Creating a node and implementing direct twist towards target

% Creating first node
dist_s = hom_dist(box_object.T, target_position.T);
node_s = create_node(1, box_object, robot, Cp_e0, ...
    Cn_e0, Cone0, true, Cp_h0, Cn_h0, zeros(1,6), dist_s, true);
    
% Implementing direct twist
[exit, nodes_out, edges_out] = ...
    implement_direct_twist2(node_s, environment, ...
    edge_types, edge_weights, target_position, 1);

disp('The exit flag is '); disp(exit);

% Drawing the path
for i = 1:height(nodes_out)
    
    % Getting the node and properties
    node_i = nodes_out(i,:); % row corresponding to r_nodeID_s
    
    % Get the properties of the start node (not the global start)
    box_i = node_i.Object{1};
    robot_i = node_i.Robot{1};
    Cp_h_i = node_i.Cp_h{1};
    Cn_h_i = node_i.Cn_h{1};
    
    % Drawing
    handle_r = robot_i.plot();
    %plot_contacts(Cp_e_s, Cn_e_s, [1 0 1]);
    plot_box(box_i.l, box_i. w, box_i.h, box_i.T, [0 0.5 0.5], true);
    plot_contacts(Cp_h_i, Cn_h_i, [1 0 1]);
    
    view(azim, elev);
    
    pause(0.5);
    
end








