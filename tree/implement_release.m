function [exit,box_f,robot_f,Cp_e_f,Cn_e_f,Cone_f,Cont_h_f,Cp_h_f,Cn_h_f] = ...
    implement_release(box_s,robot_s,Cp_e_s,Cn_e_s,Cone_s, ...
    Cont_h_s,Cp_h_s,Cn_h_s,environment,force_params)
% IMPLEMENT RELEASE - In the input node, the hand is positioned.
% Check for partial force closure of the object alone and remove hand

%   Inputs:     properties of the starting node
%              	force_params - a vector with mu k ecc...
%   Outputs:    properties of the finishing node and environment (list)
%               exit is true if a node was found, else it is false

% Some params
n_try = 3;
plot_conts = false;
plot_hand_conts = false;
plot_rob = false;
plot_obj = false;
plot_forces = false;
verbose = false;             % max time interval for moving in cone

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
    
    % Building the G, J, K, H matrices (only environment and no sliding)
    [G2, J2, K2, H2] = build_matrices_for_force(robot_s, [], [], ...
        Cp_e_s, Cn_e_s, Co_s, kh, ke, [], []);
    
    % If the above matrices are empty, don't do the following
    is_env_contacting = ~isempty(G2);
    
    if is_env_contacting
        
        % Analysis of contact point behaviour (getting contact types)
        % Here all contacts should be maintained as we give null d_pose
        [~, ~, cf_dim_e1, c_types1] = contact_type_analysis(Cp_e_f, ...
            Cn_e_f, zeros(6,1)); % Cp_e01 and Cn_e01 do not contain the detached conts.
        
        % Creating the parameters for optimization (only env)
        [normals2,mu_vect2,f_min_vect2,f_max_vect2,cf_dim_tot2] = ...
            create_params_for_optimization([], [], Cp_e_s, Cn_e_s, ...
            c_types1, [], mu_e_val, [], [], f_min_e, f_max_e);
        
        % Get particular sol. and optimize to find cont. constr. fulfilling forces
        % that also guarantee forces equilibria
        fp2 = -K2*G2.'*pinv(G2*K2*G2.')*we; % Particular solution
        
        [fc_opt2, cost_opt2, cost_init2, exitflag2, output2, elapsed_time2, ...
            sigma_leq2] = solve_constraints_particular_mincon(we, fp2, ...
            G2, K2, normals2, mu_vect2, f_min_vect2, f_max_vect2, ...
            cf_dim_tot2, Delta);
        
        if verbose
            disp('The following do not verify the constraints ');
            disp(find(sigma_leq2 > Delta));
            disp('The sum of the forces is ');
            disp(norm(we + G2*fc_opt2));
        end
        
        if (~isempty(find(sigma_leq2 > Delta)) || ...
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
    q2 = robot_f.get_release_config_george(Cp_h_s, Cn_h_s, Co_s);
    robot_f.set_config(q2);
    
    % Checking rob env collisions (COMMENTING OUT FOR ACCELERATING)
%     if robot_f.check_collisions(environment)
%         if verbose
%             disp('REL - Collision hand env detected while releasing hand');
%         end
%         break; % As of now, releasing is a simple going back (USELESS TO INSIST)
%     end
    
    % Now everything is ok
    found = true;
    
end

if ~found
    exit = false;
    return;
end

% Assigning the remanining stuff
Cone_f = Cone_s;
Cont_h_f = false;
Cp_h_f = [];
Cn_h_f = [];

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

