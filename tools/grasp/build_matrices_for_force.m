function [G,J,K,H] = build_matrices_for_force(Cp_h,Cn_h,Cp_e,Cn_e)
% BUILD MATRICES FOR FORCE - Builds the needed matrices for force analysis
% Attention: as of now we consider that all the contacts are hard finger
% only (TODO: Generalize this function)
%   Input:
%   Cp_h        - hand contact positions (rows)
%   Cn_h        - hand contact normals (rows)
%   Cp_e        - environment contact positions (rows)
%   Cn_e        - environment contact normals (rows)

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


end

