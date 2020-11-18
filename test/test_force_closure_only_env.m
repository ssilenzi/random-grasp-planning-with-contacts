%% For testing force-closure both with the environment and the hand
% Here we will test force-closure without sliding contacts

test_hand_functions;

% Reducing the environment contacts set
Cp_eout = Cp;
Cn_eout = Cn;
% [Cp_eout,Cn_eout] = minimize_contact_set(Cp,Cn,box_object);
plot_contacts(Cp_eout,Cn_eout);

% Resaving needed contact matrices
Cp_e = Cp_eout;        	% Contact positions of env to obj
Cn_e = Cn_eout;        	% Contact normals of env to obj
Cp_h = p_global;        % Contact positions of hand to obj
Cn_h = n_global;        % Contact normals of hand to obj

%% Environment Matrices

% Building matrices for environment (jac is null)
H_e = build_h(0,0,size(Cp_e,1),Cn_e); % hard finger
G_e = build_g(Cp_e, 1);
GHt_e = G_e * H_e.';
J_e = zeros(6*size(Cp_e,1), robot.get_n_dof());
HJ_e = H_e * J_e;
ke = 1000;
K_e = eye(size(H_e,1))*ke;

% Putting together
G = GHt_e;
J = HJ_e;
K = K_e;

%% Needed stuff for force-closure minimization

% Basis of Active Internal Forces
[E, dQ, dU] = basis_active_internal_forces_2(G, J, K);

% External wrench (0 for prehensility) and starting guess of int. f. vec.
% we = zeros(6,1);
we = 1*[0.71;-0.71;0;0;0;0]*9.81;
y0 = rand(size(E,2),1);

fp = -K*G.'*pinv(G*K*G.')*we; % Particular solution
fc_0 = fp + E*y0;

% Normals for the optimization function
normals = [];
cf_dim = zeros(size(Cp_e,1),1);
for i=1:size(Cp_e,1)
    cf_dim(i) = 3;
    normals = [normals; Cn_e(i,:).'];
end

% Other force constraints for optimization
num_cp = size(Cp_e,1);
mu_env = 0.5;
mu_vect = ones(1,size(Cp_e,1))*mu_env;
f_min_vect = 0.0001*ones(1,num_cp);
f_max_vect = 10000*ones(1,num_cp);

[fc_opt, y_opt, cost_opt, cost_0, exitflag, output, elapsed_time, ...
    sigma_leq, lambda] = solve_constraints_mincon(fp, ...
    normals, mu_vect, f_min_vect, f_max_vect , cf_dim, E, y0);

% Trasforming the opt_force in matrix with forces on rows and plotting
ind = 1;
Cf = [];
for i = 1: length(cf_dim)
    indf = ind+cf_dim(i)-1;
    Cf = [Cf; fc_opt(ind:indf,:).'];
    ind = indf+1;
end
plot_forces(Cp_e, Cf);

sigma_now = sigma_tot(fc_opt,normals,mu_vect, f_min_vect, f_max_vect , cf_dim);
disp('The following do not verify the constraints ');
disp(find(sigma_now > 0));

% [fc_opt, y_opt, V_opt_mincon_1, V_0, exitflag, output, elapsed_time, ...
%     sigma_leq, lambda,grad,hessian] = V_optimal_mincon(f0, normals, ...
%     mu_vect, f_min_vect, f_max_vect , cf_dim, E);


%% Elaboration of the solution
% New equilibrium variations
dustar = dU*y_opt;

% New object state
box_object_new = twist_moves_object(box_object, dustar);

% Plotting new rob-obj equilibrium
plot_box(box_object_new.l, box_object_new.w, box_object_new.h, ...
    box_object_new.T, [0 0 0], true)
