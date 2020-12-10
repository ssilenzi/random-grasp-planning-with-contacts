% Sigma Tot
%
%   Computes the constraints (to be minimized) for a contact.
%   Friction cone, minimum and maximum normal force are considered
%
%   Syntax:  [ Sigma_tot ] = Sigma_tot( f_ci, n_i, mu_i, f_min_i, f_max_i, cf_dim )
%

function [ sigma_vec ] = sigma_tot( f_c, normals, mu, f_min, f_max , m_min, m_max, cf_dim )

ind = 1;    % Index for the force stuff
indn = 1;   % Separate index for normals
sigma_temp = [];

% Checking if the hand has moments (soft finger)
% Getting the starting of the moment part (after the forces)
soft_fing = cf_dim(1) == 4;
if soft_fing
    indm = 3*length(find(cf_dim == 4));
    num_hand_conts = length(find(cf_dim == 4));
end

for i = 1: length(cf_dim)
    
    force_dim = cf_dim(i);
    if force_dim == 4
        force_dim = 3;
    end
    
    indf = ind+force_dim-1;
    indnf = indn+3-1;

    wrench_i = f_c(ind: indf);
    
    if cf_dim(i) == 4
        indmf = indm + 1;
        if i <= num_hand_conts
            wrench_i = [wrench_i; f_c(indm)];
        end
    end
    
    sigma_temp_i = sigma_i( wrench_i, ...
        normals(indn: indnf), ...
        mu(i), f_min(i), f_max(i), m_min(i), m_max(i) ) ;
    sigma_temp = [sigma_temp; sigma_temp_i];
    
    ind = indf+1;
    indn = indnf+1;
    
    if cf_dim(i) == 4
        indm = indmf + 1;
    end
    if soft_fing && i == length(find(cf_dim == 4))
        ind = ind + num_hand_conts;
    end
    
end
sigma_vec = sigma_temp ;
%
end