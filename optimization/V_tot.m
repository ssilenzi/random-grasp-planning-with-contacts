% V_tot
%
%   Computes the fucntion V (to be minimized) for the current contact force distribution.
%   Friction cone, minimum and maximum normal force are considered
%
%   Syntax: [ V_final ] = V_tot( f_c, normals, mu, f_min, f_max , cf_dim )
%
%   Inputs:
%   f_c     is a vector collecting all the current contact forces
%       Only linear forces are considered (f_ci is \in R^{3}, or \in R^{2} for planar problems).
%   normals is a vector collecting di directions of all the normals a the contact points
%       (n_i is \in R^{3}, or \in R^{2} for planar problems).
%   mu    is the vector collecting all the friction coefficient at the contacts
%   f_min is the vector collecting all the minimum normal forces admitted at the contacts
%   f_max is the vector collecting all the maximum normal forces admitted at the contacts
%
% Outputs:
% V_tot is term of the fucntion V relative to the i-th contact
%
function [ V_final ] = V_tot( f_c, normals, mu, f_min, f_max , m_min, m_max, cf_dim )
%

ind = 1;
indn = 1;
V = 0;

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
    
    V = V + V_i( wrench_i, ...
        normals(indn:indnf),...
        mu(i), f_min(i), f_max(i), m_min(i), m_max(i) ) ;
    
    ind = indf+1;
    indn = indnf+1;
    
    if cf_dim(i) == 4
        indm = indmf + 1;
    end
    if soft_fing && i == length(find(cf_dim == 4))
        ind = ind + num_hand_conts;
    end
    
end
%
V_final = V ;
%
end
%
