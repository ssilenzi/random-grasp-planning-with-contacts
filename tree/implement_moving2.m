function [exit,nodes_out,edges_out] = ...
    implement_moving2(node_s,environment,force_params, ...
    edge_types,edge_weights)

% IMPLEMENT MOVING 2 - In the input node, the hand is already positioned.
% Sample the cone for a free motion and then check for actuatability of the
% motion. Create the cone at the arrival node and return.

%   Inputs:     starting node
%               environment (list)
%              	force_params - a vector with mu k ecc...
%               info on edges
%   Outputs:    finishing nodes
%               related edges
%               exit is true if a node was found, else it is false

% Some params
n_try = 3;
verbose = false;
p_generators = 0.3;
dt = 1.0;               % max time interval for moving in cone
coll_points = 5;
hand_cont_dim = 4;      % 3 if hard finger, 4 if soft finger

% Getting the force closure related constants
mu_h_val = force_params{1}; mu_e_val = force_params{2};  
f_min_h_ac = force_params{3}; f_max_h_ac = force_params{4};
f_min_h_pf = force_params{5}; f_max_h_pf = force_params{6};
f_min_e = force_params{7}; f_max_e = force_params{8};
m_min_h = force_params{9}; m_max_h = force_params{10};
kh = force_params{11}; ke = force_params{12};
we = force_params{13};
Delta = force_params{14};

% Getting the start node properties
[ID_s, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, Cont_h_s, Cp_h_s, Cn_h_s, ...
    dir_s, ~] = get_node_properties(node_s);

% Copying already some outputs for moving (some will change, others won't)
[ID_f1, box_f1, robot_f1, Cp_e_f1, Cn_e_f1, ~ , ...
    Cont_h_f1, Cp_h_f1, Cn_h_f1, dir_f1, prev_pos_f1] = ...
    copy_node_properties(ID_s+1, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
    Cont_h_s, Cp_h_s, Cn_h_s, dir_s, false);

% Get starting object position as row
Co_s = box_s.T(1:3,4).';

% Spawning the robot in a non colliding pose after choosing random contacts
found = false;
for i = 1:n_try
    
    % Selecting a combination vec. for the cone
    p_comb = rand;
    alpha = zeros(size(Cone_s,2),1);
    if p_comb < p_generators % selecting a random generator
        ind = randsample(1:size(Cone_s,2),1);
        alpha(ind) = 1;
    else % selecting random combination of generators
        sC = size(Cone_s,2);
        for j = 1:sC
            alpha(i) = rand;
        end
    end
    
    % Checking if the new motion is not a "going back"
    if dot(dir_s,Cone_s*alpha) < 0 % choose another combination
        if verbose
            disp('MOV - Continuing, I dont want to go back!');
        end
        continue;
    end
    
    % Moving the object
    [success, box_f1, twist01, d_pose01] = get_pose_from_cone(Cone_s, ...
        box_s, environment, dt, alpha);
    if ~success || norm(d_pose01 < 0.0001)
        if verbose
            disp('MOV - Continuing, could not get a good pose inside Cone');
        end
        continue;
    end
    dir_f1 = twist01; % the new direction of motion
    
    % Checking hand-kin obj-motion compatibility
    if(~is_compatible_motion_hand_kin(robot_f,Cp_h_f,Cp_h_f,d_pose01))
        if verbose
            disp('MOV - Continuing, need to change hand contacts or object motion');
        end
        continue;
    end
    
    % Analysis of contact point behaviour (getting contact types)
    [Cp_e01, Cn_e01, ~, c_types01] = contact_type_analysis(Cp_e_s, ...
        Cn_e_s, d_pose01); % Cp_e01 and Cn_e01 do not contain the detached conts.
    
    % Building the D and N matrices and then G, J, K, H (with sliding)
    [D_tot01, N_tot01] = build_d_n(Cp_e01, Cn_e01, c_types01, d_pose01, mu_e_val);
    [G01, ~, K01, ~] = build_matrices_for_force(robot_s, Cp_h_s, Cn_h_s, ...
        Cp_e01, Cn_e01, Co_s, kh, ke, N_tot01, D_tot01, hand_cont_dim);
    
    % Creating the parameters for optimization
    [normals01,mu_vect01,f_min_vect01,f_max_vect01,cf_dim_tot01] = ...
        create_params_for_optimization(Cp_h_s, Cn_h_s, Cp_e01, Cn_e01, ...
        c_types01, mu_h_val, mu_e_val, f_min_h_ac, ...
        f_max_h_ac, f_min_e, f_max_e, m_min_h, m_max_h, hand_cont_dim);
    
    % Get particular sol. and optimize to find cont. constr. fulfilling forces
    % that also guarantee forces equilibria
    fp01 = -K01*G01.'*pinv(G01*K01*G01.')*we; % Particular solution
    
    [fc_opt01, ~, ~, ~, ~, ~, ...
        sigma_leq01] = solve_constraints_particular_mincon(we, fp01, ...
        G01, K01, normals01, mu_vect01, f_min_vect01, f_max_vect01, ...
        cf_dim_tot01, Delta);
    
    if verbose
        disp('The following do not verify the constraints ');
        disp(find(sigma_leq01 > Delta));
        disp('The sum of the forces is ');
        disp(norm(we + G01*fc_opt01));
    end
    
    if (~isempty(find(sigma_leq01 > Delta, 1)) || ...
            ~(norm(we + G01*fc_opt01) < 1e-10))
        if verbose
            disp('MOV - Continuing, motion not actuatable');
        end
        continue; % BREAK HERE, DON'T INSIST? OR CONTINUE?
    end
    
    % Finding the moved contact points and normals and new robot config
    Hom_d_pose01 = twistexp(d_pose01); % homogeneous trans. corresponding to obj twist
    Cp_h_f1 = transform_points(Cp_h_s, Hom_d_pose01);      % transforming points
    Cn_h_f1 = transform_vectors(Cn_h_s, Hom_d_pose01);     % transforming normals
    
    % Moving robot to contacts
    [robot_f1, success] = move_robot_to_points(robot_f1,Cp_h_f1);
    
    % Checking rob env collisions
    if ~success || robot_f1.check_collisions(environment,coll_points)
        if verbose
            disp('MOV - Collision hand env detected while moving hand');
        end
        continue;
    else
        if verbose
            disp('MOV - Found a good hand pose after moving');
        end
        Cont_h_f1 = true; % THERE IS OBJ-HAND CONTACT
    end
    
    % Checking for partial force closure at arrival
    % Get arrival object position as row
    Co_f1 = box_f1.T(1:3,4).';
    
    % Get contacts with the environment and plot
    [success, Cp_e_f1, Cn_e_f1] = get_contacts_with_flag(environment, box_f1, box_f1.T);
    if ~success
        if verbose
            disp('MOV - Bad edge-edge contact. get_contacts problem. Change node');
        end
        break;
    end
    
    % Analysis of contact point behaviour (getting contact types)
    % Here all contacts should be maintained as we give null d_pose
    [~, ~, ~, c_types1] = contact_type_analysis(Cp_e_f1, ...
        Cn_e_f1, zeros(6,1)); % Cp_e01 and Cn_e01 do not contain the detached conts.
    
    % Building the G, J, K, H matrices (no sliding here, so no D_tot and N_tot)
    [G1, ~, K1, ~] = build_matrices_for_force(robot_f1, Cp_h_f1, Cn_h_f1, ...
        Cp_e_f1, Cn_e_f1, Co_f1, kh, ke, [], [], hand_cont_dim);
    
    % Creating the parameters for optimization
    [normals1,mu_vect1,f_min_vect1,f_max_vect1,cf_dim_tot1] = ...
        create_params_for_optimization(Cp_h_f1, Cn_h_f1, Cp_e_f1, Cn_e_f1, ...
        c_types1, mu_h_val, mu_e_val, f_min_h_pf, f_max_h_pf, ...
        f_min_e, f_max_e, m_min_h, m_max_h, hand_cont_dim);
    
    % Get particular sol. and optimize to find cont. constr. fulfilling forces
    % that also guarantee forces equilibria
    fp1 = -K1*G1.'*pinv(G1*K1*G1.')*we; % Particular solution
    
    [fc_opt1, ~, ~, ~, ~, ~, ...
        sigma_leq1] = solve_constraints_particular_mincon(we, fp1, ...
        G1, K1, normals1, mu_vect1, f_min_vect1, f_max_vect1, ...
        cf_dim_tot1, Delta);
    
    if verbose
        disp('The following do not verify the constraints ');
        disp(find(sigma_leq1 > Delta));
        disp('The sum of the forces is ');
        disp(norm(we + G1*fc_opt1));
    end
    
    if (~isempty(find(sigma_leq1 > Delta, 1)) || ...
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

% Assigning the final cone
Cone_f1 = pfc_analysis(Cp_e_f1, Cn_e_f1, 3);

% Creating the moved node
node_mov = create_node(ID_f1, box_f1, robot_f1, Cp_e_f1, Cn_e_f1, Cone_f1, ...
    Cont_h_f1, Cp_h_f1, Cn_h_f1, dir_f1, prev_pos_f1);

% The output node
nodes_out = node_mov;

% Creating weighted edge between nodes
e_type_sf1 = edge_types{2};
e_weight_sf1 = edge_weights(2);
edge_mov = table({e_type_sf1}', e_weight_sf1, ...
    'VariableNames',{'Type','Weight'});

% Two output edges
edges_out = edge_mov;

% All is well now
exit = true;

end

