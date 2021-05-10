function [exit,nodes_out,edges_out] = ...
    implement_release2(node_s,environment,force_params, ...
    edge_types,edge_weights,target,n_nodes)

%   Inputs:     starting node
%               environment (list)
%              	force_params - a vector with mu k ecc...
%               info on edges
%               present number of nodes of the graph
%               the target object pose
%   Outputs:    finishing nodes
%               related edges
%               exit is 1 if a node was found, else it is 0

% Some params
n_try = 3;
verbose = false;
coll_points = 10;
hand_cont_dim = 4;      % 3 if hard finger, 4 if soft finger

global vec_time_cone_comp;
global vec_time_cone_red;
global vec_time_force;

% Getting the force closure related constants
mu_e_val = force_params{2};  
f_min_e = force_params{7}; f_max_e = force_params{8};
m_min_h = force_params{9}; m_max_h = force_params{10};
kh = force_params{11}; ke = force_params{12};
we = force_params{13};
Delta = force_params{14};

% Getting the start node properties
[~, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, Cont_h_s, Cp_h_s, Cn_h_s, ...
    dir_s, dist_s, ~] = get_node_properties(node_s);

% Copying already some outputs for moving (some will change, others won't)
[ID_f1, box_f1, robot_f1, Cp_e_f1, Cn_e_f1, ~ , ...
    ~, ~, ~, dir_f1, dist_f1, prev_pos_f1] = ...
    copy_node_properties(n_nodes+1, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
    Cont_h_s, Cp_h_s, Cn_h_s, dir_s, dist_s, false);

% Assigning at first empty nodes_out and edges_out
nodes_out = [];
edges_out = [];

% Get starting object position as row
Co_s = box_s.T(1:3,4).';

% Spawning the robot in a non colliding pose after choosing random contacts
found = false;
for i = 1:n_try
    
    % Building the G, J, K, H matrices (only environment and no sliding)
    [G2, ~, K2, ~] = build_matrices_for_force(robot_s, [], [], ...
        Cp_e_s, Cn_e_s, Co_s, kh, ke, [], [], hand_cont_dim);
    
    % If the above matrices are empty, don't do the following
    is_env_contacting = ~isempty(G2);
    
    if is_env_contacting
        
        % Analysis of contact point behaviour (getting contact types)
        % Here all contacts should be maintained as we give null d_pose
        [~, ~, ~, c_types1] = contact_type_analysis(Cp_e_s, ...
            Cn_e_s, zeros(6,1)); % Cp_e01 and Cn_e01 do not contain the detached conts.
        
        % Creating the parameters for optimization (only env)
        [normals2, mu_vect2, f_min_vect2, f_max_vect2, ...
            m_min_vect2, m_max_vect2, cf_dim_tot2] = ...
            create_params_for_optimization([], [], Cp_e_s, Cn_e_s, ...
            c_types1, [], mu_e_val, [], [], f_min_e, f_max_e, ...
            m_min_h, m_max_h, hand_cont_dim);
        
        % Get particular sol. and optimize to find cont. constr. fulfilling forces
        % that also guarantee forces equilibria
        fp2 = -K2*G2.'*pinv(G2*K2*G2.')*we; % Particular solution
        
        tic;
        [fc_opt2, ~, ~, ~, ~, ~, ...
            sigma_leq2] = solve_constraints_particular_mincon(we, fp2, ...
            G2, K2, normals2, mu_vect2, f_min_vect2, f_max_vect2, ...
            m_min_vect2, m_max_vect2, cf_dim_tot2, Delta);
        vec_time_force = [vec_time_force toc];
        
        if verbose
            disp('The following do not verify the constraints ');
            disp(find(sigma_leq2 > Delta));
            disp('The sum of the forces is ');
            disp(norm(we + G2*fc_opt2));
        end
        
        if (~isempty(find(sigma_leq2 > Delta, 1)) || ...
                ~(norm(we + G2*fc_opt2) < 1e-10))
            if verbose
                disp('REL - Continuing, release not possible');
            end
            break; % BREAK HERE, USELESS TO INSIST!
        end
        
    else
        if verbose
            disp('REL - Only hand contacts: cannot release. Change node');
        end
        break;
    end
    
    % Moving the robot to a release configuration
    sig2 = robot_f1.get_release_config_george(Cp_h_s, Cn_h_s, Co_s);
    robot_f1.set_act(sig2);
    
    % Checking rob env collisions
    if robot_f1.check_collisions(environment, coll_points)
        if verbose
            disp('REL - Collision hand env detected while releasing hand');
        end
        break; % As of now, releasing is a simple going back (USELESS TO INSIST)
    end
    
    % Now everything is ok
    found = true;
    
end

if ~found
    exit = 0;
    return;
end

% Assigning the remanining stuff
Cone_f1 = Cone_s;
Cont_h_f1 = false;
Cp_h_f1 = [];
Cn_h_f1 = [];

% Creating the moved node
node_rel = create_node(ID_f1, box_f1, robot_f1, Cp_e_f1, Cn_e_f1, Cone_f1, ...
    Cont_h_f1, Cp_h_f1, Cn_h_f1, dir_f1, dist_f1, prev_pos_f1);

% The output node
nodes_out = node_rel;

% Creating weighted edge between nodes
e_type_sf1 = edge_types{3};
e_weight_sf1 = edge_weights(3);
edge_rel = table({e_type_sf1}', e_weight_sf1, ...
    'VariableNames',{'Type','Weight'});

% Two output edges
edges_out = edge_rel;

% All is well now
exit = 1;

end

