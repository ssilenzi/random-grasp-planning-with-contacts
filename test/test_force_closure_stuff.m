%% For testing force-closure both with the environment and the hand
% Here we will test force-closure without sliding contacts

test_hand_functions;

% Resaving needed contact matrices
Cp_e = Cp;          % Contact positions of env to obj
Cn_e = Cn;          % Contact normals of env to obj
Cp_h = p_global;    % Contact positions of hand to obj
Cn_h = n_global;    % Contact normals of hand to obj

%% Hand + environment Matrices

% Building matrices for hand
H_h = build_h(0,0,size(Cp_h,1),Cn_h); % hard finger
G_h = build_g(Cp_h, 1);
GHt_h = G_h * H_h.';
J_h = robot.get_jacobian();
HJ_h = H_h * J_h;
K_h = H_h*robot.get_joint_contact_stiffness()*H_h.'; % to apply H

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
% G = G_e;
% J = J_e;
% K = K_e;

%% Needed stuff for force-closure minimization

% Basis of Active Internal Forces
[E, dQ, dU] = basis_active_internal_forces_2(G, J, K);

% External wrench (0 for prehensility)
we = zeros(6,1);
% we = [0;-1;0;0;0;-1]*9.81;

f0 = -K*G.'*pinv(G*K*G.')*we; % Particular solution

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
f_min_vect = 0.1*ones(1,num_cp);
f_max_vect = 100*ones(1,num_cp);

V_0 = V_tot(f0, normals, mu_vect, f_min_vect, f_max_vect , cf_dim); % E_el

[fc_opt, y, V_opt_mincon_1, V_0, exitflag, output, elapsed_time, ...
    sigma_leq, lambda,grad,hessian] = V_optimal_mincon(f0, normals, ...
    mu_vect, f_min_vect, f_max_vect , cf_dim, E);

% [fc_opt, y_opt, V_opt, V_0, exit_tests , elapsed_time, ...
%     sigma_leq, grad_opt, Hessian_opt, V_vect] = V_optimal_Newton( f0, ... 
%     normals, mu_vect, f_min_vect, f_max_vect , cf_dim, E , []);


