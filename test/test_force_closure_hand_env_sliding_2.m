%% For testing force-closure both with the environment and the hand
% Here we will test force-closure without sliding contacts

test_hand_functions_sliding;

% mu_hand_val = 3; mu_env_val = 3;
% mu_hand_val = 3; mu_env_val = 0.1; % Works always, too optimistic
% mu_hand_val = 1.5; mu_env_val = 0.5; %
mu_hand_val = 0.5; mu_env_val = 0.1; % 

% Reducing the environment contacts set
Cp_eout = Cp;
Cn_eout = Cn;
% [Cp_eout,Cn_eout] = minimize_contact_set(Cp,Cn,box_object);
% plot_contacts(Cp_eout,Cn_eout);

% Resaving needed contact matrices
Cp_e = Cp_eout;        	% Contact positions of env to obj
Cn_e = Cn_eout;        	% Contact normals of env to obj
Cp_h = p_global;        % Contact positions of hand to obj
Cn_h = n_global;        % Contact normals of hand to obj

%% Hand + environment Matrices

% Building matrices for hand
H_h = build_h(0,0,size(Cp_h,1),Cn_h); % hard finger
G_h = build_g_cont(Cp_h, Co, 1);
GHt_h = G_h * H_h.';
J_h = robot.get_jacobian();
HJ_h = H_h * J_h;
kh = 1000;
K_h = eye(size(H_h,1))*kh;

% Building matrices for environment (jac is null)
H_e = build_h(0,0,size(Cp_e,1),Cn_e); % hard finger
G_e = build_g_cont(Cp_e, Co, 1);
GHt_e = G_e * H_e.';
J_e = zeros(6*size(Cp_e,1), robot.get_n_dof());
HJ_e = H_e * J_e;
ke = 1000;
K_e = eye(size(H_e,1))*ke;

% Putting together
G = [GHt_h GHt_e];
J = [HJ_h; HJ_e];
K = blkdiag(K_h,K_e);

%% Verify that Hand kinematics is compatible with object twist
% Find the motion of the contact points of the hand caused by the object
% twist and check if it is within the range of HJ (actuatable using q).
% If the contact motions do not cause a motion of the hand joints
% (the hand cannot actuae that motion), then we discard the corresonding
% object twist

c_h =  (GHt_h.')*twist;

if rank(HJ_h) ~= rank([HJ_h, c_h])
    disp('Twist is not compatible with hand kinematics');
    return;
end

%% Finding the moved contact points and normals and new robot config
Hom_twist = twistexp(twist); % homogeneous trans. corresponding to obj twist
Cp_h2 = transform_points(Cp_h, Hom_twist);      % transforming points
Cn_h2 = transform_vectors(Cn_h, Hom_twist);     % transforming normals
plot_contacts(Cp_h2, Cn_h2, [1 0 1]);

% Finding a robot config for the new pose
q_now = robot.q;
% Getting the current wrist pose
x_now = robot.get_forward_kinematics();
x_wrist = x_now(:,:,3);

% Setting the desired points and IK
xd(:,:,1) = [eye(3) Cp_h2(1,1:3).'; [0 0 0 1]];
xd(:,:,2) = [eye(3) Cp_h2(2,1:3).'; [0 0 0 1]];
xd(:,:,3) = x_wrist;
q_open_d = q_now(7:8);

robot.compute_differential_inverse_kinematics_george(xd, q_open_d);

handle3 = robot.plot();

% New object contacts
[Cp_e2, Cn_e2] = get_contacts(environment, box_object2, box_object2.T);
plot_contacts(Cp_e2,Cn_e2);

%% Analysis of contact point behaviour (using starting contacts)

% Getting the type of contacts and f vectors dimensions
cf_e_dim = [];  % Dimensions of the force vector
c_types = [];   % Type (1 - maint, 2 - det, 3 - ?, 4 - slide)
Cp_e_prime = Cp_e;
Cn_e_prime = Cn_e;
for i=1:size(Cp_e,1)
    disp(fprintf('Contact %d:', i));
    
    Cp_e_i = Cp_e(i,:); % get the contact i
    Cn_e_i = Cn_e(i,:);
    
    GG_e_i = build_g(Cp_e_i, 1);
    H_e_i = build_h(0,0,1,Cn_e_i);
    c_e_p_i = H_e_i*GG_e_i'*twist;
    if(truncate(norm(c_e_p_i)) == 0)
        cf_e_dim = [cf_e_dim 3];
        disp('    maintained');
        c_types = [c_types; 1];
    elseif (truncate(Cn_e_i*c_e_p_i) > 0)
        disp('    detached');
        c_types = [c_types; 2];
    elseif (truncate(Cn_e_i*c_e_p_i) < 0)
        disp('    WARNING - to be debugged - cont-env compenetration?');
        c_types = [c_types; -1];
    else
        cf_e_dim = [cf_e_dim 1];
        disp('    slipping');
        c_types = [c_types; 3];
    end
    
end

% Removing the detached indexes from c_types and related contact normal and position
indexes_det = find(c_types == 2); % CHECK IF THIS CHANGE WORKS
Cp_e_prime(indexes_det,:) = [];
Cn_e_prime(indexes_det,:) = [];
c_types(indexes_det) = [];


%% Build matrix D and N for contacts according to type (maintained or sliding)
mu_env = mu_env_val;

D_tot = [];
N_tot = [];

for i=1:size(Cp_e_prime,1)
    % Getting the ith contact position and normal
    Cp_e_i = Cp_e_prime(i,:);
    Cn_e_i = Cn_e_prime(i,:);
    
    % Different implementations according to contact types
    if(c_types(i) == 1) % maintained
        D_i = eye(3);
        N_i = eye(3);
    elseif (c_types(i) == 3) % sliding
        G_s_i = build_g(Cp_e_i, 1);
        H_s_i = build_h(0,0,1,Cp_e_i);
        c_e_p_i = H_s_i*G_s_i'*twist;   % Getting the contact velocity (we know it is orth to normal)
        n_i = Cn_e_i';                  % Normal of the contact
        D_i = n_i - mu_env*c_e_p_i/norm(c_e_p_i);   % d_i for sliding (look presentation)
        N_i = n_i;
%         disp('c_e_p_i '); disp(c_e_p_i);
%         disp('N_i '); disp(N_i);
%         disp('D_i '); disp(D_i);
    elseif (c_types(i) == 2) % detached
        % THIS WON'T HAPPEN AS THE DETACHED ARE NO MORE CONSIDERED HERE
        disp('    WARNING - What??? Detached were already removed!');
    elseif (c_types(i) == -1) % compenetration?
        disp('    WARNING - to be debugged - cont-env compenetration?');
    else
        error('    ERROR - this value for c_types is not admissible!');
    end
    
    % Augmenting D_tot and N_tot with the computed D_i and N_i
    D_tot = blkdiag(D_tot, D_i);
    N_tot = blkdiag(N_tot, N_i);
end


%% Building the new basic matrices taking into account also sliding

% Re-building new matrices for environment (detached conts not considered)
H_e = build_h(0,0,size(Cp_e_prime,1),Cn_e_prime); % hard finger
G_e = build_g_cont(Cp_e_prime, Co, 1);
GHt_e = G_e * H_e.';
J_e = zeros(6*size(Cp_e_prime,1), robot.get_n_dof());
HJ_e = H_e * J_e;
ke = 1000;

% Putting together
G = [GHt_h, GHt_e*N_tot];
J = [HJ_h; N_tot.'*HJ_e];
K_h = eye(size(H_h,1))*kh;
K_e = eye(size(N_tot.',1))*ke;
K = blkdiag(K_h,K_e);

%% Needed stuff for force-closure minimization

% Basis of Active Internal Forces
[E, dQ, dU] = basis_active_internal_forces_2(G, J, K);

% External wrench (0 for prehensility) and starting guess of int. f. vec.
% we = zeros(6,1);
we = 0.1*[0;-1;0;0;0;0]*9.81; % Attention here that this is expressed obj frame
y0 = rand(size(E,2),1);

plot_forces([-5 10 -5], we.'); % Plotting gravity

fp = -K*G.'*pinv(G*K*G.')*we; % Particular solution

% Normals for the optimization function
normals = [];
cf_dim_tot = [];
num_cp = 0;     % Total no. of contacts
for i=1:size([Cp_h;Cp_e_prime],1)
    if i <= size(Cp_h,1)
        normals =  [normals; Cn_h(i,:).'];
        cf_dim_tot = [cf_dim_tot, 3];
    else
        if c_types(i-size(Cp_h,1)) == 1
            cf_dim_tot = [cf_dim_tot, 3];
        elseif c_types(i-size(Cp_h,1)) == 3
            cf_dim_tot = [cf_dim_tot, 1];
        end
           normals = [normals; Cn_e_prime(i -size(Cp_h,1),:).'];
    end
    num_cp = num_cp+1;
end

% Other force constraints for optimization
mu_hand = mu_hand_val;
mu_env = mu_env_val;
mu_vect = [ones(1,size(Cp_h,1))*mu_hand ones(1,size(Cp_e_prime,1))*mu_env];
f_min_vect = 0*ones(1,num_cp); f_min_vect(1:2) = 0.5*ones(1,2);
f_max_vect = 5*ones(1,num_cp);
Delta = 0.00005;

% First an optimization of fp to get a good one
[fp_sol, cost_sol, cost_0, exitflag, output, elapsed_time, ...
    sigma_leq] = solve_constraints_particular_mincon(we, fp, ...
    G, K, normals, mu_vect, f_min_vect, f_max_vect , cf_dim_tot, Delta);

% Then using also the act. int. basis
% [fc_opt, y_opt, cost_opt, cost_0, exitflag, output, elapsed_time, ...
%     sigma_leq] = solve_constraints_internal_mincon(fp_sol, E, y0, ...
%     we, G, normals, mu_vect, f_min_vect, f_max_vect , cf_dim_tot, Delta);

fc_opt = fp_sol;

% Now the normal comp. of the sliding forces shall be trasformed so as to
% get the whole forces including the tangential component too.
Cp_tot = [Cp_h; Cp_e_prime];
Cn_tot = [Cn_h; Cn_e_prime];
fc_opt_tot = [];
ind = 1;
for i = 1: size(Cn_tot,1)
    fc_tmp = [];
    hand_ind = size(Cn_h,1);
    if i <= hand_ind
        indf = ind+3-1;
        fc_tmp = fc_opt(ind:indf,:);
    else
        if c_types(i-hand_ind) == 1
            indf = ind+3-1;
            fc_tmp = fc_opt(ind:indf,:);
        elseif c_types(i-hand_ind) == 3
            indf = ind+cf_e_dim(i-hand_ind)-1;
            n_i = Cn_tot(i,:).';
            p_i = Cp_tot(i,:);
            G_s_i = build_g(p_i, 1);
            H_s_i = build_h(0,0,1,p_i);
            c_e_p_i = H_s_i*G_s_i'*twist;
            norm_force = fc_opt(ind:indf,:)*n_i;
            tang_force = - mu_env * norm(norm_force)*(c_e_p_i / norm(c_e_p_i));
            fc_tmp = norm_force + tang_force;
        else
            error('Dont know what to do here! Not supposed to get this!');
        end
    end
    fc_opt_tot = [fc_opt_tot; fc_tmp];
    ind = indf+1;
end 

% Trasforming the opt_force in matrix with forces on rows and plotting
ind = 1;
Cf = [];
for i = 1: size(Cn_tot,1)
    indf = ind+3-1;
    Cf = [Cf; fc_opt_tot(ind:indf,:).'];
    ind = indf+1;
end
plot_forces(Cp_tot, Cf);

disp('The following do not verify the constraints ');
disp(find(sigma_leq > Delta));

% Checking which forces do not comply with the constraints
indexes_viol = find(sigma_leq > 0.00005);
ind = 1;
Cp_viol = [];
Cf_viol = [];
for i = 1:size(Cf,1)
    indf = ind+3-1;
    if any(ismember([ind:indf],indexes_viol)) 
        Cf_viol = [Cf_viol; Cf(i,:)];
        Cp_viol = [Cp_viol; Cp_tot(i,:)];
    end
    ind = indf+1;
end

% Plotting only Object, Hand and Forces
figure;
plot_boxes({box_object}, true);
plot_forces(Cp_tot, Cf);
plot_forces(Cp_viol, Cf_viol, [1 0 0]);
plot_points_color(Cp_viol, [1 0 0]);
axis([-15 15 -15 15 -15 15]); % Change the axis and view
view(50, 30);


