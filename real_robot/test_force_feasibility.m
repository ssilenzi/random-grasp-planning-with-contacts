%% For answering reviewer 2/7

%% Cleaning up and defining constants
close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

addpath('../grippers/franka_collision_check');

% Define main constants
dt_max = 0.1;          	% max dt for getting a new pose from velocity cone
hand_cont_dim = 4;      % 3 if hard finger, 4 if soft finger
do_aux_plots = true;    % for plotting extra stuff
start_moved = false;  	% to start from a moved pose
n_try = 50;             % Max num. of tries for finding collision free pose
dm_to_m = 0.5e-1;

% Define force related constants
mu_h_val = 0.7; mu_e_val = 0.2;     % friction constants
f_min_h_ac = 0.5; f_max_h_ac = 5;  	% max and min hand force norms for actuatability
f_min_h_pf = 0; f_max_h_pf = 5;  	% max and min hand force norms for force feasibility
f_min_e = 0; f_max_e = 2;           % max and min env force norms
m_min_h = 0; m_max_h = 1;           % max and min hand moment norms
kh = 1000; ke = 1000;              	% contact stiffness
we = 0.1*[0;0;-1;0;0;0]*9.81;      	% Attention here that we should be expressed obj frame

% Define optimization params
Delta = 0.0000;    % a margin for extra robustness

%% Building scenario, object and hand
% Loading the hand
robot_name = 'franka_emika_panda';
franka = load_gripper(robot_name);
fig_h = figure('Color',[1 1 1], 'pos',[0 0 800 800], ...
    'WindowState', 'maximized');
rob_h = franka.plot([], false, gca);

% Build the scenario and the box
% run('franka_book_vertical_empty.m')
% run('franka_book_on_table_vertical.m')
% run('franka_book_on_shelf.m')
% run('franka_book_on_table_cluttered.m')
% run('franka_book_on_table_vertical_cluttered.m')
% run('franka_cp_book_on_table_vertical.m')
run('franka_cp_books_on_kallax.m')
tot_h = plot_scenario(environment,box_object, ...
    target_position,axis_range,azim,elev);

% plot_forces([0 0 1.5], we.',[0 0.5 0.5],dm_to_m); % Plotting gravity force

%% Getting cone and sampling
% Get object position as row
Co0 = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp_e0, Cn_e0] = get_contacts(environment, box_object, box_object.T);

% Getting the cone and plotting if necessary
Cone0 = pfc_analysis(Cp_e0, Cn_e0, 3);

% Selecting a combination vec. and moving the object
ind = randsample(1:size(Cone0,2),1);
alpha0 = zeros(size(Cone0,2),1); alpha0(ind) = 1; %alpha0(5) = 1; % selecting a generator

[success, box_obj1, twist01, d_pose01] = get_pose_from_cone(Cone0, ...
    box_object, environment, dt_max, alpha0);

if ~success
    error('Could not get a good pose inside Cone');
end

plot_box(box_obj1.l, box_obj1. w, box_obj1.h, box_obj1.T, [0 1 1], true);

%% Moving robot to collision free random points and checking obj. motion compatibility
rob_coll = true;

for i = 1:n_try
    
%     disp(i);
    % Getting random contacts on free faces
    [Cp_h0, Cn_h0] = get_random_contacts_on_box_partial(box_object, ...
        franka.n_contacts, Cp_e0, Cn_e0, true, dm_to_m);
    
    franka1 = copy(franka);
    
    % Loading the hand in a starting pose
    [q0, success] = franka1.get_starting_config(Cp_h0, Cn_h0, Co0, box_object);
    rob_h = franka1.plot([], false, gca);
    if ~success
        warning('Did not get good IK sol for pregrasp!');
%         franka1.q > franka1.up_joint_lims
%         franka1.q < franka1.lo_joint_lims
%         franka1.q
    end
    % Moving robot to contacts and checking for collisions
    [franka1, success] = move_franka_to_points(franka1,Cp_h0);
    [bool_self_col, self_coll_pair_id, world_coll_pair_id] = ...
        franka1.build_check_collisions(box_object, environment);
    rob_h = franka1.plot([], true, gca);
    if ~success
        warning('Did not get good IK sol for grasp!');
%         franka1.q > franka1.up_joint_lims
%         franka1.q < franka1.lo_joint_lims
%         franka1.q
    end    
%     if self_coll_pair_id
%         warning('Did not get good pose for grasp because self collision!');
%     end
%     if world_coll_pair_id
%         warning('Did not get good pose for grasp because world collision!');
%     end
    if ~success %|| ~isempty(self_coll_pair_id) || ~isempty(world_coll_pair_id)
        warning('No good contacts! Changing!');
        % go further with the next random points
    else
        disp('Found a good hand pose');
        rob_coll = false;
        break; % the first ntry that is ok, is the way to go
    end
    
end

if(rob_coll)
    error('Cannot go on here, did not find and rob env coll free pose');
end

franka = franka1;

% Checking hand-kin obj-motion compatibility
if(~is_compatible_motion_hand_kin(franka,Cp_h0,Cn_h0,d_pose01))
    error('Cannot go on here, need to change hand contacts or object motion');
end

%% Checking for actuatability of the motion
% Analysis of contact point behaviour (getting contact types)
[Cp_e01, Cn_e01, cf_dim_e01, c_types01] = contact_type_analysis(Cp_e0, ...
    Cn_e0, d_pose01); % Cp_e01 and Cn_e01 do not contain the detached conts.

% Building the D and N matrices and then G, J, K, H (with sliding)
[D_tot01, N_tot01] = build_d_n(Cp_e01, Cn_e01, c_types01, d_pose01, mu_e_val);
[G01, J01, K01, H01] = build_matrices_for_force(franka, Cp_h0, Cn_h0, ...
    Cp_e01, Cn_e01, Co0, kh, ke, N_tot01, D_tot01, hand_cont_dim);

% Creating the parameters for optimization
[normals01,mu_vect01,f_min_vect01,f_max_vect01,m_min_vect01,m_max_vect01,cf_dim_tot01] = ...
    create_params_for_optimization(Cp_h0, Cn_h0, Cp_e01, Cn_e01, ...
    c_types01, mu_h_val, mu_e_val, f_min_h_ac, f_max_h_ac, ...
    f_min_e, f_max_e, m_min_h, m_max_h, hand_cont_dim);


% Get particular sol. and optimize to find cont. constr. fulfilling forces
% that also guarantee forces equilibria
fp01 = -K01*G01.'*pinv(G01*K01*G01.')*we; % Particular solution
[E, dQ, dU] = basis_active_internal_forces_2(G01, J01, K01);
y0 = zeros(size(E,2),1);

tic
[fc_opt01, y_opt01, cost_opt01, cost_init01, exitflag01, output01, elapsed_time01, ...
    sigma_leqm01] = solve_constraints_full_mincon_correct(fp01, ...
    E, y0, normals01, mu_vect01, f_min_vect01, f_max_vect01, ...
    m_min_vect01, m_max_vect01, cf_dim_tot01, Delta);
toc

disp('The following do not verify the constraints ');
disp(find(sigma_leqm01 > Delta));
disp('The sum of the forces is ');
disp(norm(we + G01*fc_opt01));

%% Finding the moved contact points and normals and new robot config
% Finding the moved contact points and normals and new robot config
Hom_d_pose01 = twistexp(d_pose01); % homogeneous trans. corresponding to obj twist
Cp_h1 = transform_points(Cp_h0, Hom_d_pose01);      % transforming points
Cn_h1 = transform_vectors(Cn_h0, Hom_d_pose01);     % transforming normals
plot_contacts(Cp_h1, Cn_h1, [1 0 1], dm_to_m);

% Wanted wrist transform
wrist0 = franka.get_forward_kinematics();
wrist0 = wrist0(1:4,4,3); % previous wrist position
wrist1 = Hom_d_pose01 * wrist0;
wrist1 = wrist1(1:3);

% Moving the robot with the wanted wrist transform
[franka_new, success] = move_franka_to_points(franka,Cp_h1,wrist1);
% rob_h = franka_new.plot([], true, gca);
if ~success
    warning('Did not get good IK sol for moving!');
    franka_new.q > franka_new.up_joint_lims
    franka_new.q < franka_new.lo_joint_lims
    franka_new.q
end
