function [exit,box_f,robot_f,Cp_e_f,Cn_e_f,Cone_f,Cont_h_f,Cp_h_f,Cn_h_f] = ...
    implement_moving(box_s,robot_s,Cp_e_s,Cn_e_s,Cone_s, ...
    Cont_h_s,Cp_h_s,Cn_h_s,environment,force_params)
% IMPLEMENT MOVING - In the input node, the hand is already positioned.
% Sample the cone for a free motion and then check for actuatability of the
% motion. Create the cone at the arrival node and return.

%   Inputs:     properties of the starting node
%              	force_params - a vector with mu k ecc...
%   Outputs:    properties of the finishing node and environment (list)
%               exit is true if a node was found, else it is false

% Some params
n_try = 10;
plot_conts = false;
plot_hand_conts = false;
plot_rob = false;
plot_obj = false;
plot_forces = false;
verbose = false;
generators = true;
dt = 1.0;               % max time interval for moving in cone

% Getting the force closure related constants
mu_h_val = force_params{1}; mu_e_val = force_params{2};  
f_min_h_ac = force_params{3}; f_max_h_ac = force_params{4};
f_min_h_pf = force_params{5}; f_max_h_pf = force_params{6};
f_min_e = force_params{7}; f_max_e = force_params{8};
kh = force_params{9}; ke = force_params{10};
we = force_params{11};
Delta = force_params{12};

% Assigning already some outputs (some will change, others won't)
box_f = box_s;
robot_f = copy(robot_s);
Cp_e_f = Cp_e_s;
Cn_e_f = Cn_e_s;
Cone_f = Cone_s;
Cont_h_f = Cont_h_s; % this one is false but will become true
Cp_h_f = Cp_h_s;
Cn_h_f = Cn_h_s;

% Get starting object position as row
Co_s = box_s.T(1:3,4).';

% Spawning the robot in a non colliding pose after choosing random contacts
found = false;
for i = 1:n_try
    
    % Selecting a combination vec. for the cone
    alpha = [];
    if generators
        % selecting a random generator
        ind = randsample([1:size(Cone_s,2)],1);
        alpha = zeros(size(Cone_s,2),1); alpha(ind) = 1;
    else
        % TODO
        error('This part is not implemented yet!');
    end
    
    % Moving the object
    [success, box_f, twist01, d_pose01] = get_pose_from_cone(Cone_s, ...
        box_s, environment, dt, alpha);
    if ~success
        if verbose
            disp('MOV - Continuing, could not get a good pose inside Cone');
        end
        continue;
    end
    
    % Checking hand-kin obj-motion compatibility
    if(~is_compatible_motion_hand_kin(robot_s,Cp_h_s,Cp_h_s,d_pose01))
        if verbose
            disp('MOV - Continuing, need to change hand contacts or object motion');
        end
        continue;
    end
    
    % Analysis of contact point behaviour (getting contact types)
    [Cp_e01, Cn_e01, cf_dim_e01, c_types01] = contact_type_analysis(Cp_e_s, ...
        Cn_e_s, d_pose01); % Cp_e01 and Cn_e01 do not contain the detached conts.
    
    % Building the D and N matrices and then G, J, K, H (with sliding)
    [D_tot01, N_tot01] = build_d_n(Cp_e01, Cn_e01, c_types01, d_pose01, mu_e_val);
    [G01, J01, K01, H01] = build_matrices_for_force(robot_s, Cp_h_s, Cn_h_s, ...
        Cp_e01, Cn_e01, Co_s, kh, ke, N_tot01, D_tot01);
    
    % Creating the parameters for optimization
    [normals01,mu_vect01,f_min_vect01,f_max_vect01,cf_dim_tot01] = ...
        create_params_for_optimization(Cp_h_s, Cn_h_s, Cp_e01, Cn_e01, ...
        c_types01, mu_h_val, mu_e_val, f_min_h_ac, f_max_h_ac, f_min_e, f_max_e);
    
    % Get particular sol. and optimize to find cont. constr. fulfilling forces
    % that also guarantee forces equilibria
    fp01 = -K01*G01.'*pinv(G01*K01*G01.')*we; % Particular solution
    
    [fc_opt01, cost_opt01, cost_init01, exitflag01, output01, elapsed_time01, ...
        sigma_leq01] = solve_constraints_particular_mincon(we, fp01, ...
        G01, K01, normals01, mu_vect01, f_min_vect01, f_max_vect01, ...
        cf_dim_tot01, Delta);
    
    if verbose
        disp('The following do not verify the constraints ');
        disp(find(sigma_leq01 > Delta));
        disp('The sum of the forces is ');
        disp(norm(we + G01*fc_opt01));
    end
    
    if (~isempty(find(sigma_leq01 > Delta)) || ...
            ~(norm(we + G01*fc_opt01) < 1e-10))
        if verbose
            disp('MOV - Continuing, motion not actuatable');
        end
        continue; % BREAK HERE, DON'T INSIST? OR CONTINUE?
    end
    
    if plot_forces
        [fc_opt_tot01,Cf01,Cp_viol01,Cf_viol01] = ...
            post_process_forces(Cp_h0, Cn_h0, Cp_e01, Cn_e01, d_pose01, ...
            fc_opt01, c_types01, cf_dim_e01, sigma_leq01, Delta, mu_e_val);
        
        % Plotting the forces on the main figure
        Cp_tot01 = [Cp_h0; Cp_e01];
        % plot_forces(Cp_tot01, Cf01);
    end
    
    % Finding the moved contact points and normals and new robot config
    Hom_d_pose01 = twistexp(d_pose01); % homogeneous trans. corresponding to obj twist
    Cp_h_f = transform_points(Cp_h_s, Hom_d_pose01);      % transforming points
    Cn_h_f = transform_vectors(Cn_h_s, Hom_d_pose01);     % transforming normals
    
    % Moving robot to contacts
    [robot_f, success] = move_robot_to_points(robot_f,Cp_h_f);
    
    % Checking rob env collisions
    if ~success || robot_f.check_collisions(environment)
        if verbose
            disp('MOV - Collision hand env detected while moving hand');
        end
        continue;
    end
    
    % Checking for partial force closure at arrival
    % Get new object position as row
    Co_f = box_f.T(1:3,4).';
    
    % Get contacts with the environment and plot
    [success, Cp_e_f, Cn_e_f] = get_contacts_with_flag(environment, box_f, box_f.T);
    if ~success
        if verbose
            disp('MOV - Bad edge-edge contact. get_contacts problem. Change node');
        end
        break;
    end
    
    % Analysis of contact point behaviour (getting contact types)
    % Here all contacts should be maintained as we give null d_pose
    [~, ~, cf_dim_e1, c_types1] = contact_type_analysis(Cp_e_f, ...
        Cn_e_f, zeros(6,1)); % Cp_e01 and Cn_e01 do not contain the detached conts.
    
    % Building the G, J, K, H matrices (no sliding here, so no D_tot and N_tot)
    [G1, J1, K1, H1] = build_matrices_for_force(robot_f, Cp_h_f, Cn_h_f, ...
        Cp_e_f, Cn_e_f, Co_f, kh, ke, [], []);
    
    % Creating the parameters for optimization
    [normals1,mu_vect1,f_min_vect1,f_max_vect1,cf_dim_tot1] = ...
        create_params_for_optimization(Cp_h_f, Cn_h_f, Cp_e_f, Cn_e_f, ...
        c_types1, mu_h_val, mu_e_val, f_min_h_pf, f_max_h_pf, f_min_e, f_max_e);
    
    % Get particular sol. and optimize to find cont. constr. fulfilling forces
    % that also guarantee forces equilibria
    fp1 = -K1*G1.'*pinv(G1*K1*G1.')*we; % Particular solution
    
    [fc_opt1, cost_opt1, cost_init1, exitflag1, output1, elapsed_time1, ...
        sigma_leq1] = solve_constraints_particular_mincon(we, fp1, ...
        G1, K1, normals1, mu_vect1, f_min_vect1, f_max_vect1, ...
        cf_dim_tot1, Delta);
    
    if verbose
        disp('The following do not verify the constraints ');
        disp(find(sigma_leq1 > Delta));
        disp('The sum of the forces is ');
        disp(norm(we + G1*fc_opt1));
    end
    
    if (~isempty(find(sigma_leq1 > Delta)) || ...
            ~(norm(we + G1*fc_opt1) < 1e-10))
        if verbose
            disp('MOV - Continuing, no partial force closure at arrival');
        end
        continue; % BREAK HERE, DON'T INSIST? OR CONTINUE?
    else
        % At this point all is good: break
        if verbose
            disp('MOV - Good moving found!');
        end
        found = true;
        break;
    end
    
end

if ~found
    exit = false;
    return;
end

% Assigning the remanining stuff
Cone_f = pfc_analysis(Cp_e_f, Cn_e_f, 3);
Cont_h_f = Cont_h_s;

% Plotting stuff
if plot_conts
    plot_contacts(Cp_e_f, Cn_e_f, [1 0 1]);
end
if plot_rob
    rob_handle1 = robot_f.plot();
end
if plot_obj
    plot_box(box_f.l, box_f. w, box_f.h, box_f.T, [0 0 0], true);
end
if plot_hand_conts
    plot_contacts(Cp_h_f, Cn_h_f, [1 0 1]);
end

exit = true;

end

