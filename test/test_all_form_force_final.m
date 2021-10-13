%% FOR TESTING ALL MAIN STUFF %%
% (hand func., par. form-closure, forces distr., constr. compliance)
% integrating also partially covered face contacts and collisions

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

% Define force related constants
mu_h_val = 0.7; mu_e_val = 0.2;     % friction constants
f_min_h_ac = 0.5; f_max_h_ac = 5;  	% max and min hand force norms for actuatability
f_min_h_pf = 0; f_max_h_pf = 5;  	% max and min hand force norms for par. force closure
f_min_e = 0; f_max_e = 2;           % max and min env force norms
m_min_h = 0; m_max_h = 1;           % max and min hand moment norms
kh = 1000; ke = 1000;              	% contact stiffness
we = 0.1*[0;-1;0;0;0;0]*9.81;      	% Attention here that we should be expressed obj frame

% Define optimization params
Delta = 0.00005;    % a small positive margin for aiding convergence of the
% optimization with fmincon; used in checking sigmas

%% Building scenario, object and hand
% Build the scenario and the box (only initial pose)
% run('book_vertical_empty.m')
% run('book_on_table.m')
run('book_on_table_vertical.m')
% run('book_on_box_corner_no_target.m')
% run('book_on_shelf_no_other_books.m')
% run('book_on_shelf_no_target.m')
% run('book_on_table_cluttered_no_target.m')

axis(axis_range); axis equal; % Change the axis and view
view(azim, elev);
legend off;

plot_forces([-5 10 -5], we.'); % Plotting gravity force

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
ind = randsample([1:size(Cone0,2)],1);
alpha0 = zeros(size(Cone0,2),1); alpha0(ind) = 1; %alpha0(5) = 1; % selecting a generator
tic
[success, box_obj1, twist01, d_pose01] = get_pose_from_cone(Cone0, ...
    box_object, environment, dt, alpha0);
toc
if ~success
    error('Could not get a good pose inside Cone');
end

%% If starting moved, move the object and redo the cone
if start_moved
    % Get object position as row
    Coint = box_obj1.T(1:3,4).';
    
    % Get contacts with the environment and plot
    [Cp_eint, Cn_eint] = get_contacts(environment, box_obj1, box_obj1.T);
    
    % Getting the cone and plotting if necessary
    Coneint = pfc_analysis(Cp_e0, Cn_e0, 3);
    
    % Moving again the object with the same alpha
    [success, box_obj_new, twist01, d_pose01] = get_pose_from_cone( ...
        Coneint, box_obj1, environment, dt, alpha0);
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

% Plotting the initial stuff
plot_contacts(Cp_e0, Cn_e0);
% plot_free_cone(Cone0,dt,box_object,all_boxes,axis_range,azim,elev);
plot_box(box_obj1.l, box_obj1. w,box_obj1.h, box_obj1.T, [0 0 0], true);

%% Moving robot to collision free random points and checking obj. motion compatibility
rob_coll = true;
tic
for i = 1:n_try
    
%     disp(i);
    % Getting random contacts on free faces
    [Cp_h0, Cn_h0] = get_random_contacts_on_box_partial(box_object, num_hand_conts, ...
        Cp_e0, Cn_e0, true);
    
    % Loading the hand in a starting pose
    sig0 = robot.get_starting_config_george(Cp_h0, Cn_h0, Co0);
    robot.set_act(sig0);
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
toc

if(rob_coll)
    error('Cannot go on here, did not find and rob env coll free pose');
end

tic
% Checking hand-kin obj-motion compatibility
if(~is_compatible_motion_hand_kin(robot,Cp_h0,Cn_h0,d_pose01))
    error('Cannot go on here, need to change hand contacts or object motion');
end
toc

%% Checking for actuatability of the motion
% Analysis of contact point behaviour (getting contact types)
[Cp_e01, Cn_e01, cf_dim_e01, c_types01] = contact_type_analysis(Cp_e0, ...
    Cn_e0, d_pose01); % Cp_e01 and Cn_e01 do not contain the detached conts.

% Building the D and N matrices and then G, J, K, H (with sliding)
[D_tot01, N_tot01] = build_d_n(Cp_e01, Cn_e01, c_types01, d_pose01, mu_e_val);
[G01, J01, K01, H01] = build_matrices_for_force(robot, Cp_h0, Cn_h0, ...
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

[fc_opt_tot01,Cf01,Cp_viol01,Cf_viol01] = ...
    post_process_forces(Cp_h0, Cn_h0, Cp_e01, Cn_e01, d_pose01, ...
    fc_opt01, c_types01, cf_dim_tot01, sigma_leq01, Delta, mu_e_val);

% Plotting the forces on the main figure
Cp_tot01 = [Cp_h0; Cp_e01];
% plot_forces(Cp_tot01, Cf01);

%% Finding the moved contact points and normals and new robot config
Hom_d_pose01 = twistexp(d_pose01); % homogeneous trans. corresponding to obj twist
Cp_h1 = transform_points(Cp_h0, Hom_d_pose01);      % transforming points
Cn_h1 = transform_vectors(Cn_h0, Hom_d_pose01);     % transforming normals
plot_contacts(Cp_h1, Cn_h1, [1 0 1]);

% Wanted wrist transform
wrist0 = robot.get_forward_kinematics();
wrist0 = wrist0(1:4,4,3); % previous wrist position
wrist1 = Hom_d_pose01 * wrist0;
wrist1 = wrist1(1:3);

% Moving the robot with the wanted wrist transform
[robot, success] = move_robot_to_points_and_wrist(robot,Cp_h1,wrist1);
if ~success
    warning('The hand was not moved correctly!');
end
rob_handle1 = robot.plot();

%% Checking for partial force closure at arrival
% Get new object position as row
Co1 = box_obj1.T(1:3,4).';

% Get contacts with the environment and plot
[Cp_e1, Cn_e1] = get_contacts(environment, box_obj1, box_obj1.T);
plot_contacts(Cp_e1, Cn_e1);

% Analysis of contact point behaviour (getting contact types)
% Here all contacts should be maintained as we give null d_pose
[~, ~, cf_dim_e1, c_types1] = contact_type_analysis(Cp_e1, ...
    Cn_e1, zeros(6,1)); % Cp_e01 and Cn_e01 do not contain the detached conts.

% Building the G, J, K, H matrices (no sliding here, so no D_tot and N_tot)
[G1, J1, K1, H1] = build_matrices_for_force(robot, Cp_h1, Cn_h1, ...
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
[G2, J2, K2, H2] = build_matrices_for_force(robot, [], [], ...
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
sig2 = robot.get_release_config_george(Cp_h1, Cn_h1, Co1);
robot.set_act(sig2);
rob_handle2 = robot.plot();

%% Plotting the forces on separate figures
% Moving object
figure;
plot_boxes({box_object}, true);
plot_forces(Cp_tot01, Cf01);
% plot_forces(Cp_viol01, Cf_viol01, [1 0 0]); NOT WORKING AS OF NOW
% plot_points_color(Cp_viol01, [1 0 0]); NOT WORKING AS OF NOW
axis(axis_range); % Change the axis and view
view(azim, elev);

% Arrival
figure;
plot_boxes({box_obj1}, true);
plot_forces(Cp_tot1, Cf1);
% plot_forces(Cp_viol1, Cf_viol1, [1 0 0]); NOT WORKING AS OF NOW
% plot_points_color(Cp_viol1, [1 0 0]); NOT WORKING AS OF NOW
axis(axis_range); % Change the axis and view
view(azim, elev);

% Release
if is_env_contacting
    figure;
    plot_boxes({box_obj1}, true);
    plot_forces(Cp_tot2, Cf2);
%     plot_forces(Cp_viol2, Cf_viol2, [1 0 0]); NOT WORKING AS OF NOW
%     plot_points_color(Cp_viol2, [1 0 0]); NOT WORKING AS OF NOW
    axis(axis_range); % Change the axis and view
    view(azim, elev);
end

