function [res, fc_opt, exitflag, c_types] = ...
    is_twist_actuatable(t, object, robot, Cp_e, Cn_e, Cp_h, Cn_h, we, ke, ...
    mu_env, mu_hand)

res = true;
return;

if ~exist('we', 'var')
    we = zeros(6,1);
end
if ~exist('se', 'var')
    ke = 1000;
end

if ~exist('mu_env', 'var')
    mu_env = 0.5;
end
if ~exist('mu_hand', 'var')
    mu_hand = 0.5;
end
% ISTWISTACTUATABLE This function evaluates if a twist "t" can be executed
% by a robot in its configuration "robot_state"


%% Robot Hand Model


GG_h = build_g(Cp_h, 1);
JJt_h = robot.get_jacobian().';


%% Veify that Hand kinematics is compatible with twist
H_h = build_h(0,0,robot.get_n_contacts(),Cn_h);


G_h = (H_h*GG_h.').';
Jt_h = (H_h*JJt_h.').';

J_h = Jt_h.';
P_h = (Jt_h*J_h)\Jt_h;
tmp = Jt_h.'*P_h;
I = eye(size(tmp));
res1 = truncate((I - J_h*P_h)*G_h.'*t);

if norm(res1) ~= 0
    disp('Twist t is not compatible with hand kinematics');
    res = false;
    return;
end

c_types = zeros(size(Cp_e,1),1);

D = [];
N = [];

for i = 1:size(Cp_h,1)
    D = blkdiag(D, eye(3));
    N = blkdiag(N, eye(3));
end
arg_m = 1e-3;
for i=1:size(Cp_e,1)
    
    Cp_e_i = Cp_e(i,:); % get the contact i
    Cn_e_i = Cn_e(i,:);
    
    GG_e_i = build_g(Cp_e_i, 1);
    H_e_i = build_h(0,0,1,Cn_e_i);
    c_e_p = H_e_i*GG_e_i.'*t;
    if(truncate(norm(c_e_p)) == 0) % maintained contacts
        c_types(i) = 1;
        D = blkdiag(D, eye(3));
        N = blkdiag(N, eye(3).');
    elseif (truncate(Cn_e_i*c_e_p) > 0) % detached contacts
        c_types(i) = 2;
    elseif (truncate(Cn_e_i*c_e_p) < 0)
        disp('    WARNING - to be debugged');
    else % sliding contacts
        c_types(i) = 3;
        D = blkdiag(D, (Cn_e_i - mu_env*Cp_e_i/norm(Cp_e_i)).');
        N = blkdiag(N, Cn_e_i);
    end
    
end

Cp_e = Cp_e(c_types ~= 2,:);
Cn_e = Cn_e(c_types ~= 2,:);
c_types_reduced = c_types(c_types ~= 2);

G = build_g([Cp_h;Cp_e], 1);
H = build_h(0,0,size([Cp_h;Cp_e],1),[Cn_h;Cn_e]);

K_e = eye(size(Cn_e(c_types_reduced == 1,:),1)*3 + ...
    size(Cn_e(c_types_reduced == 3,:),1))*ke;
K_h = H_h*robot.get_joint_contact_stiffness()*H_h.'; % to apply H

K = blkdiag(K_h,K_e);
J = [robot.get_jacobian();zeros(size(c_types_reduced,1)*6,robot.get_n_dof())];
S = robot.get_synergies();
% E = getMatrixE(J,G*H'*D',S,K);

% K is weighted differently are the new weigths D*K*N

f0 = -K*N*H*G.'*(pinv(G*H.'*D*K*N*H*G.')*we);

E = orth((eye(size(f0,1)) - K*N*H*G.'*(pinv(G*H'*D*K*N*H*G.'))*G*H.'*D)*...
    K*N*H*J);

normals = [];
cf_dim = zeros(size([Cp_h;Cp_e],1),1);
for i=1:size([Cp_h;Cp_e],1)
    if i <= size(Cp_h,1)
        cf_dim(i) = 3;
        normals =  [normals; Cn_h(i,:).'];
    else
        if c_types_reduced(i -size(Cp_h,1)) == 1
           cf_dim(i) = 3;
           normals = [normals; Cn_e(i -size(Cp_h,1),:).'];
        elseif c_types_reduced(i -size(Cp_h,1)) == 3
            cf_dim(i) = 1;
            normals = [normals;1];
        end
    end
end
    
num_cp = size(Cp_e,1) + size(Cn_h,1);
% mu_vect = ones(1,num_cp)*mu_env;
mu_vect = [ones(1,size(Cp_h,1))*mu_hand ones(1,size(c_types_reduced,1))*...
    mu_env];
% num_cp = num_cp + size(Cn_h,1); 
f_min_vect = 0.1*ones(1,num_cp);
f_max_vect = 100*ones(1,num_cp);

V_0 = V_tot(f0, normals, mu_vect, f_min_vect, f_max_vect , cf_dim); % E_el

[fc_opt, y,V_opt_mincon_1,V_0,exitflag,output, elapsed_time, sigma_leq, ...
    lambda,grad,hessian] = V_optimal_mincon(f0, normals, mu_vect, ...
    f_min_vect, f_max_vect , cf_dim, E);

index = 1;

c_types = [ones(size(Cn_h,1),1);c_types];

for i=1:size(c_types,1)
    if c_types(i) == 2
        continue;
    end
    if c_types(i) == 1
        f = fc_opt(index:index+2);
        if(~(norm(f) < f_max_vect(i) && norm(f) > f_min_vect(i)))
           res = false;
           return;
        end
        index =  index+3;
    end
    if c_types(i) == 3
        index = index + 1;
    end

end

if exitflag >= 0
    res = true;
end
end