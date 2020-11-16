%% For testing force-closure both with the environment and the hand
% Here we will test force-closure without sliding contacts

test_hand_functions;

% Reducing the environment contacts set
[Cp_eout,Cn_eout] = minimize_contact_set(Cp,Cn,box_object);
plot_contacts(Cp_eout,Cn_eout);

% Resaving needed contact matrices
Cp_e = Cp_eout;        	% Contact positions of env to obj
Cn_e = Cn_eout;        	% Contact normals of env to obj
Cp_h = p_global;        % Contact positions of hand to obj
Cn_h = n_global;        % Contact normals of hand to obj

%% Hand + environment Matrices

% Building matrices for hand
H_h = build_h(0,0,size(Cp_h,1),Cn_h); % hard finger
G_h = build_g(Cp_h, 1);
GHt_h = G_h * H_h.';
J_h = robot.get_jacobian();
HJ_h = H_h * J_h;
kh = 1000;
K_h = eye(size(H_h,1))*kh;

% Building matrices for environment (jac is null)
H_e = build_h(0,0,size(Cp_e,1),Cn_e); % hard finger
G_e = build_g(Cp_e, 1);
GHt_e = G_e * H_e.';
J_e = zeros(6*size(Cp_e,1), robot.get_n_dof());
HJ_e = H_e * J_e;
ke = 1000;
K_e = eye(size(H_e,1))*ke;

% Putting together
G = [GHt_h GHt_e];
J = [HJ_h; HJ_e];
K = blkdiag(K_h,K_e);

%% Needed stuff for force-closure minimization

% Basis of Active Internal Forces
[E, dQ, dU] = basis_active_internal_forces_2(G, J, K);

% External wrench (0 for prehensility) and starting guess of int. f. vec.
we = zeros(6,1);
% we = [0;-1;0;0;0;-1]*9.81;
y0 = rand(size(E,2),1);

fp = -K*G.'*pinv(G*K*G.')*we; % Particular solution
fc_0 = fp + E*y0;

% Normals for the optimization function
normals = [];
cf_dim = zeros(size([Cp_h;Cp_e],1),1);
for i=1:size([Cp_h;Cp_e],1)
    if i <= size(Cp_h,1)
        cf_dim(i) = 3;
        normals =  [normals; Cn_h(i,:).'];
    else
           cf_dim(i) = 3;
           normals = [normals; Cn_e(i -size(Cp_h,1),:).'];
    end
end

% Other force constraints for optimization
num_cp = size(Cp_e,1) + size(Cp_h,1);
mu_hand = 0.5;
mu_env = 0.5;
mu_vect = [ones(1,size(Cp_h,1))*mu_hand ones(1,size(Cp_e,1))*mu_env];
f_min_vect = 0.0001*ones(1,num_cp);
f_max_vect = 10000*ones(1,num_cp);

% V_0 = V_tot( f0, normals, mu_vect, f_min_vect, f_max_vect , cf_dim ) ;
% grad_V = D_V_tot( f0, normals, mu_vect, f_min_vect, f_max_vect , cf_dim, E  ); % D_V_tot( f_c, normals, mu, f_min, f_max , cf_dim, E  ) ;
% Hessian_V = H_V_tot( f0, normals, mu_vect, f_min_vect, f_max_vect , cf_dim, E  );
% 
% disp(V_0);
% disp(grad_V);
% disp(Hessian_V);

% [fc_opt, y_opt, V_opt, V_0, exitflag, output, elapsed_time, ...
%     sigma_leq, lambda, grad, hessian] = V_optimal_global_mincon(fp, ...
%     normals, mu_vect, f_min_vect, f_max_vect , cf_dim, E, y0);

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
plot_forces([Cp_h; Cp_e], Cf);

%% Elaboration of the solution
% New equilibrium variations
dustar = dU*y_opt;
dqstar = dQ*y_opt;

% New object state
box_object_new = twist_moves_object(box_object, dustar);

% New robot config
qstar = robot.q + dqstar;
robot.set_config(qstar);

% Plotting new rob-obj equilibrium
plot_box(box_object_new.l, box_object_new.w, box_object_new.h, ...
    box_object_new.T, [0 0 0], true)
handle3 =robot.plot();

