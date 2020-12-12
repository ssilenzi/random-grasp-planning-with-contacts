function [G,J,K,H] = build_matrices_for_force(robot,Cp_h,Cn_h,Cp_e,Cn_e,Co,kh,ke,N_tot,D_tot)
% BUILD MATRICES FOR FORCE - Builds the needed matrices for force analysis
% Attention: as of now we consider that all the contacts are hard finger
% only (TODO: Generalize this function)
%   Inputs:
%   robot      	- robot with all its props and funcs (to get jacobian)
%   Cp_h        - hand contact positions (rows)
%   Cn_h        - hand contact normals (rows)
%   Cp_e        - environment contact positions (rows)
%   Cn_e        - environment contact normals (rows)
%   Co          - object position (row)
%   kh, ke      - contact stiffnesses of hand and environment
%   D_tot       - D matrix for generalization to sliding case
%   N_tot       - N matrix of normals (for generalization to sliding case)
%   Outputs
%   G,J,K,H     - main matrices for grasp force analysis

% TODO: Here maybe a dimensions check would be nice
do_hand = [];
if ~isempty(Cp_h) && ~isempty(Cn_h)
    do_hand = true;
elseif isempty(Cp_h) && isempty(Cn_h)
    do_hand = false;
else
    error('You have provided only one or none between Cp_h and Cn_h');
end
do_env = [];
if ~isempty(Cp_e) && ~isempty(Cn_e)
    do_env = true;
elseif isempty(Cp_e) && isempty(Cn_e)
    do_env = false;
else
    error('You have provided only one or none between Cp_e and Cn_e');
end

% Building matrices for hand
if do_hand 
    % These H matrices are already in global frame
    H_h = build_h(0,0,size(Cp_h,1),Cn_h); % hard finger
%     H_h = build_h(0,size(Cp_h,1),0,Cn_h) % soft finger
%     H_h = build_h(size(Cp_h,1),0,0,Cn_h); % fully constrained finger
    G_h = build_g_cont(Cp_h, Co, 1);
    GHt_h = G_h * H_h.';
    J_h = robot.get_jacobian();
    HJ_h = H_h * J_h;
else
    H_h = [];
    GHt_h = [];
    HJ_h = [];
end

% Building matrices for environment (jac is null)
if do_env
    H_e = build_h(0,0,size(Cp_e,1),Cn_e); % hard finger
    G_e = build_g_cont(Cp_e, Co, 1);
    GHt_e = G_e * H_e.';
    J_e = zeros(6*size(Cp_e,1), robot.get_n_dof());
    HJ_e = H_e * J_e;
else
    H_e = [];
    GHt_e = [];
    HJ_e = [];
end

% If N_tot and D_tot are not provided, set them as identity
if isempty(D_tot)
    D_tot = eye(size(H_e,1));
    if ~do_env
       D_tot = []; 
    end
end
if isempty(N_tot)
    N_tot = eye(size(H_e,1));
    if ~do_env
       N_tot = []; 
    end
end

% Building contact compliance matrices
K_h = eye(size(H_h,1))*kh;
K_e = eye(size(N_tot.',1))*ke;

% Putting together
G = [GHt_h GHt_e*D_tot];
J = [HJ_h; N_tot.'*HJ_e];
K = blkdiag(K_h,K_e);
H = blkdiag(H_h,H_e);


end

