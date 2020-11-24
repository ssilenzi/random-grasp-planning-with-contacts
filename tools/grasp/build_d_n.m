function [D_tot,N_tot] = build_d_n(Cp_e_prime,Cn_e_prime,c_types,d_pose,mu_env)
% BUILD D N - Builds the matrices D and N taking into account also the
% case of sliding
%   Inputs:
%   Cp_e_prime  - env. contact positions (rows) without the detached
%   Cn_e_prime  - env. contact normals (rows) without the detached
%   d_pose      - variation of the object pose
%   mu_env      - environement friction constant
%   c_types     - vec. showing the types of contacts (check function
%               contact_type_analysis for more details)
%   Outputs:
%   D_tot       - D matrix for generalization to sliding case
%   N_tot       - N matrix of normals (for generalization to sliding case)


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
        c_e_p_i = H_s_i*G_s_i'*d_pose;   % Getting the contact velocity (we know it is orth to normal)
        n_i = Cn_e_i';                  % Normal of the contact
        D_i = n_i - mu_env*c_e_p_i/norm(c_e_p_i);   % d_i for sliding (look presentation)
        N_i = n_i;
%         disp('c_e_p_i '); disp(c_e_p_i);
%         disp('N_i '); disp(N_i);
%         disp('D_i '); disp(D_i);
    elseif (c_types(i) == 2) % detached
        % THIS WON'T HAPPEN AS THE DETACHED ARE NO MORE CONSIDERED HERE
        warn('What??? Detached should be already removed!');
    elseif (c_types(i) == -1) % compenetration?
        warn('To be debugged - cont-env compenetration?');
    else
        error('This value for c_types is not admissible!');
    end
    
    % Augmenting D_tot and N_tot with the computed D_i and N_i
    D_tot = blkdiag(D_tot, D_i);
    N_tot = blkdiag(N_tot, N_i);
end

end

