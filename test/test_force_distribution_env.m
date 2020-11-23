%% For testing force-closure both with the environment and the hand
% Here we will test force-closure without sliding contacts

test_contacts_env;

mu_hand_val = 0.5; mu_env_val = 0.1;
% mu_hand_val = 3; mu_env_val = 0.1;

% Reducing the environment contacts set
Cp_eout = Cp;
Cn_eout = Cn;
% [Cp_eout,Cn_eout] = minimize_contact_set(Cp,Cn,box_object);
plot_contacts(Cp_eout,Cn_eout);

% Resaving needed contact matrices
Cp_e = Cp_eout;        	% Contact positions of env to obj
Cn_e = Cn_eout;        	% Contact normals of env to obj

%% Environment Matrices

% Building matrices for environment (jac is null)
H_e = build_h(0,0,size(Cp_e,1),Cn_e); % hard finger
G_e = build_g_cont(Cp_e, Co, 1);
GHt_e = G_e * H_e.';
ke = 1000;
K_e = eye(size(H_e,1))*ke;

% Putting together
G = GHt_e;
K = K_e;

%% Here we shall compute the particular solution
% This shall be done to be already compliant with the contact constraints
% as E is of no use when only env contacts

% External wrench and starting guess of fp
we = 0.3*[0;-1;1;0;0;0]*9.81; % Attention here that this is expressed obj frame

plot_forces([-5 10 -5], we.'); % Plotting gravity

% fp_guess = rand(size(-K*G.'*pinv(G*K*G.')*we)) - 0.5; % Particular solution
fp_guess = -K*G.'*pinv(G*K*G.')*we; % Particular solution

% Normals for the optimization function
normals = [];
cf_dim = zeros(size(Cp_e,1),1);
for i=1:size(Cp_e,1)
    cf_dim(i) = 3;
    normals = [normals; Cn_e(i,:).'];
end

% Other force constraints for optimization
num_cp = size(Cp_e,1);
mu_env = mu_env_val;
mu_vect = ones(1,size(Cp_e,1))*mu_env;
f_min_vect = 0*ones(1,num_cp);
f_max_vect = 2*ones(1,num_cp);
Delta = 0.00005;

[fp_sol, cost_sol, cost_0, exitflag, output, elapsed_time, ...
    sigma_leq] = solve_constraints_particular_mincon(we, fp_guess, ...
    G, K, normals, mu_vect, f_min_vect, f_max_vect , cf_dim, Delta);

% Trasforming the opt_force in matrix with forces on rows and plotting
ind = 1;
Cf = [];
for i = 1: length(cf_dim)
    indf = ind+cf_dim(i)-1;
    Cf = [Cf; fp_sol(ind:indf,:).'];
    ind = indf+1;
end
plot_forces(Cp_e, Cf);

disp('The following do not verify the constraints ');
disp(find(sigma_leq > Delta));
disp('The sum of the forces is ');
disp(norm(we + G*fp_sol));

% Checking which forces do not comply with the constraints
indexes_viol = find(sigma_leq > Delta);
ind = 1;
Cp_viol = [];
Cf_viol = [];
for i = 1:size(Cf,1)
    indf = ind+3-1;
    if any(ismember([ind:indf],indexes_viol)) 
        Cf_viol = [Cf_viol; Cf(i,:)];
        Cp_viol = [Cp_viol; Cp_e(i,:)];
    end
    ind = indf+1;
end

% Plotting only Object, Hand and Forces
figure;
plot_boxes({box_object}, true);
plot_forces(Cp_e, Cf);
plot_forces(Cp_viol, Cf_viol, [1 0 0]);
plot_points_color(Cp_viol, [1 0 0]);
axis([-10 10 -15 15 -15 15]); % Change the axis and view
view(50, 30);
