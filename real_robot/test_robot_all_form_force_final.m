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
hand_cont_dim = 4;      % 3 if hard finger, 4 if soft finger
do_aux_plots = true;    % for plotting extra stuff
start_moved = false;  	% to start from a moved pose
n_try = 50;             % Max num. of tries for finding collision free pose
dm_to_m = 0.5e-1;

% Define force related constants
mu_h_val = 0.7; mu_e_val = 0.2;     % friction constants
f_min_h_ac = 0.5; f_max_h_ac = 5;  	% max and min hand force norms for actuatability
f_min_h_pf = 0; f_max_h_pf = 5;  	% max and min hand force norms for par. force closure
f_min_e = 0; f_max_e = 2;           % max and min env force norms
m_min_h = 0; m_max_h = 1;           % max and min hand moment norms
kh = 1000; ke = 1000;              	% contact stiffness
we = 0.1*[0;0;-1;0;0;0]*9.81;      	% Attention here that we should be expressed obj frame

% Define optimization params
Delta = 0.00005;    % a small positive margin for aiding convergence of the
% optimization with fmincon; used in checking sigmas

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
% run('franka_cp_books_on_kallax.m')
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

%% If starting moved, move the object and redo the cone
if start_moved
    % Get object position as row
    Coint = box_obj1.T(1:3,4).';
    
    % Get contacts with the environment and plot
    [Cp_eint, Cn_eint] = get_contacts(environment, box_obj1, box_obj1.T);
    
    % Getting the cone and plotting if necessary
    Coneint = pfc_analysis(Cp_eint, Cn_eint, 3);
    
    % Selecting a combination vec. and moving the object
    ind = randsample(1:size(Coneint,2),1);
    alpha0 = zeros(size(Coneint,2),1); alpha0(ind) = 1; %alpha0(5) = 1; % selecting a generator
    [success, box_obj_new, twist01, d_pose01] = get_pose_from_cone( ...
        Coneint, box_obj1, environment, dt_max, alpha0);
    if ~success
        error('Could not get a good pose inside Cone');
    end
    
    % Updating the needed info
    box_object = box_obj1;
    box_obj1 = box_obj_new;
    Co0 = Coint;
    Cp_e0 = Cp_eint; Cn_e0 = Cn_eint;
    Cone0 = Coneint;
end

% Plotting the initial  and moved object stuff
plot_contacts(Cp_e0, Cn_e0, [0 1 0], dm_to_m);
% plot_free_cone(Cone0,dt,box_object,all_boxes,axis_range,azim,elev);
plot_box(box_obj1.l, box_obj1.w, box_obj1.h, box_obj1.T, [0 0 0], true);

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
    if self_coll_pair_id
        warning('Did not get good pose for grasp because self collision!');
    end
    if world_coll_pair_id
        warning('Did not get good pose for grasp because world collision!');
    end
    if ~success || ~isempty(self_coll_pair_id) || ~isempty(world_coll_pair_id)
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

tic

[fc_opt01, cost_opt01, cost_init01, exitflag01, output01, elapsed_time01, ...
    sigma_leq01] = solve_constraints_particular_mincon(we, fp01, ...
    G01, K01, normals01, mu_vect01, f_min_vect01, f_max_vect01, ...
    m_min_vect01, m_max_vect01, cf_dim_tot01, Delta);

toc

disp('The following do not verify the constraints ');
disp(find(sigma_leq01 > Delta));
disp('The sum of the forces is ');
disp(norm(we + G01*fc_opt01));

% Checking if the forces are actively executable by the hand
[success,y_star,dq_star,du_star] = ...
    is_executable_by_hand(fc_opt01,we,cf_dim_tot01,G01,J01,K01,Delta);

if ~success
    error('Force not executable by hand!');
end

[fc_opt_tot01,Cf01,Cp_viol01,Cf_viol01] = ...
    post_process_forces(Cp_h0, Cn_h0, Cp_e01, Cn_e01, d_pose01, ...
    fc_opt01, c_types01, cf_dim_tot01, sigma_leq01, Delta, mu_e_val);

% Plotting the forces on the main figure
Cp_tot01 = [Cp_h0; Cp_e01];
% plot_forces(Cp_tot01, Cf01);

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
[franka, success] = move_franka_to_points(franka,Cp_h1,wrist1);
rob_h = franka.plot([], true, gca);
if ~success
    warning('Did not get good IK sol for moving!');
    franka.q > franka.up_joint_lims
    franka.q < franka.lo_joint_lims
    franka.q
end

%% Checking for partial force closure at arrival
% Get new object position as row
Co1 = box_obj1.T(1:3,4).';

% Get contacts with the environment and plot
[Cp_e1, Cn_e1] = get_contacts(environment, box_obj1, box_obj1.T);
plot_contacts(Cp_e1, Cn_e1, [0 1 0], dm_to_m);

% Analysis of contact point behaviour (getting contact types)
% Here all contacts should be maintained as we give null d_pose
[~, ~, cf_dim_e1, c_types1] = contact_type_analysis(Cp_e1, ...
    Cn_e1, zeros(6,1)); % Cp_e01 and Cn_e01 do not contain the detached conts.

% Building the G, J, K, H matrices (no sliding here, so no D_tot and N_tot)
[G1, J1, K1, H1] = build_matrices_for_force(franka, Cp_h1, Cn_h1, ...
    Cp_e1, Cn_e1, Co1, kh, ke, [], [], hand_cont_dim);

% Creating the parameters for optimization
[normals1,mu_vect1,f_min_vect1,f_max_vect1,m_min_vect1,m_max_vect1,cf_dim_tot1] = ...
    create_params_for_optimization(Cp_h1, Cn_h1, Cp_e1, Cn_e1, ...
    c_types1, mu_h_val, mu_e_val, f_min_h_pf, f_max_h_pf, ...
    f_min_e, f_max_e, m_min_h, m_max_h, hand_cont_dim);

% Get particular sol. and optimize to find cont. constr. fulfilling forces
% that also guarantee forces equilibria
fp1 = -K1*G1.'*pinv(G1*K1*G1.')*we; % Particular solution

tic

[fc_opt1, cost_opt1, cost_init1, exitflag1, output1, elapsed_time1, ...
    sigma_leq1] = solve_constraints_particular_mincon(we, fp1, ...
    G1, K1, normals1, mu_vect1, f_min_vect1, f_max_vect1, ...
    m_min_vect1, m_max_vect1, cf_dim_tot1, Delta);

toc

disp('The following do not verify the constraints ');
disp(find(sigma_leq1 > Delta));
disp('The sum of the forces is ');
disp(norm(we + G1*fc_opt1));

[fc_opt_tot1,Cf1,Cp_viol1,Cf_viol1] = ...
    post_process_forces(Cp_h1, Cn_h1, Cp_e1, Cn_e1, zeros(6,1), ...
    fc_opt1, c_types1, cf_dim_tot1, sigma_leq1, Delta, mu_e_val);

% Plotting the forces on the main figure
Cp_tot1 = [Cp_h1; Cp_e1];
% plot_forces(Cp_tot1, Cf1);

%% Checking for partial force closure for removal
% Building the G, J, K, H matrices (only environment and no sliding)
[G2, J2, K2, H2] = build_matrices_for_force(franka, [], [], ...
    Cp_e1, Cn_e1, Co1, kh, ke, [], [], hand_cont_dim);

% If the above matrices are empty, don't do the following
is_env_contacting = ~isempty(G2);

if is_env_contacting
    
    % Creating the parameters for optimization (only env)
    [normals2,mu_vect2,f_min_vect2,f_max_vect2,m_min_vect2,m_max_vect2,cf_dim_tot2] = ...
        create_params_for_optimization([], [], Cp_e1, Cn_e1, ...
        c_types1, [], mu_e_val, [], [], f_min_e, f_max_e, ...
        m_min_h, m_max_h, hand_cont_dim);
    
    % Get particular sol. and optimize to find cont. constr. fulfilling forces
    % that also guarantee forces equilibria
    fp2 = -K2*G2.'*pinv(G2*K2*G2.')*we; % Particular solution
    
    tic
    
    [fc_opt2, cost_opt2, cost_init2, exitflag2, output2, elapsed_time2, ...
        sigma_leq2] = solve_constraints_particular_mincon(we, fp2, ...
        G2, K2, normals2, mu_vect2, f_min_vect2, f_max_vect2, ...
        m_min_vect2, m_max_vect2, cf_dim_tot2, Delta);
    
    toc
    
    disp('The following do not verify the constraints ');
    disp(find(sigma_leq2 > Delta));
    disp('The sum of the forces is ');
    disp(norm(we + G2*fc_opt2));
    
    [fc_opt_tot2,Cf2,Cp_viol2,Cf_viol2] = ...
        post_process_forces([], [], Cp_e1, Cn_e1, zeros(6,1), ...
        fc_opt2, c_types1, cf_dim_tot2, sigma_leq2, Delta, mu_e_val);
    
    % Plotting the forces on the main figure
    Cp_tot2 = Cp_e1;
    % plot_forces(Cp_tot2, Cf2);
    
end

%% Moving the robot to a release configuration
[q2, success] = franka.get_release_config(Cp_h1, Cn_h1, Co1, box_obj1);
rob_h = franka.plot([], true, gca);

%% Plotting the forces on separate figures
% Moving object
figure;
plot_boxes({box_object}, true);
plot_forces(Cp_tot01, Cf01, [0 0.5 0.5], dm_to_m);
% plot_forces(Cp_viol01, Cf_viol01, [1 0 0]); NOT WORKING AS OF NOW
% plot_points_color(Cp_viol01, [1 0 0]); NOT WORKING AS OF NOW
% axis(axis_range); % Change the axis and view
view(azim, elev);

% Arrival
figure;
plot_boxes({box_obj1}, true);
plot_forces(Cp_tot1, Cf1, [0 0.5 0.5], dm_to_m);
% plot_forces(Cp_viol1, Cf_viol1, [1 0 0]); NOT WORKING AS OF NOW
% plot_points_color(Cp_viol1, [1 0 0]); NOT WORKING AS OF NOW
% axis(axis_range); % Change the axis and view
view(azim, elev);

% Release
if is_env_contacting
    figure;
    plot_boxes({box_obj1}, true);
    plot_forces(Cp_tot2, Cf2, [0 0.5 0.5], dm_to_m);
%     plot_forces(Cp_viol2, Cf_viol2, [1 0 0]); NOT WORKING AS OF NOW
%     plot_points_color(Cp_viol2, [1 0 0]); NOT WORKING AS OF NOW
%     axis(axis_range); % Change the axis and view
    view(azim, elev);
end

