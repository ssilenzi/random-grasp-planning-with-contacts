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
function [ V_final ] = V_tot( f_c, normals, mu, f_min, f_max , cf_dim )
%

ind = 1;
V = 0;
for i = 1: length(cf_dim)
   indf = ind+cf_dim(i)-1;
   V = V + V_i( f_c(ind:indf), ...
                     normals(ind:indf),...
                     mu(i), f_min(i), f_max(i) ) ;
   ind = indf+1;
end
%
V_final = V ;
%
end
%
