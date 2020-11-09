% H_V_tot
%
%   Computes the Hessina of the function V (to be minimized) for the current contact force distribution.
%   Friction cone, minimum and maximum normal force are considered
%
%   Syntax: [ Hess_V_tot ] = H_V_tot( f_c, normals, mu, f_min, f_max , cf_dim, E  )
%
%   Inputs:
%   f_c     is a vector collecting all the current contact forces
%       Only linear forces are considered (f_ci is \in R^{3}, or \in R^{2} for planar problems). 
%   normals is a vector collecting di directions of all the normals a the contact points
%       (n_i is \in R^{3}, or \in R^{2} for planar problems). 
%   mu    is the vector collecting all the friction coefficient at the contacts
%   f_min is the vector collecting all the minimum normal forces admitted at the contacts
%   f_max is the vector collecting all the maximum normal forces admitted at the contacts
%   cf_dim \in R^{ (numb. of chains)} = the i-th element specifies the number of
%          the contact constraint on the i-th contact point
%   E = basis of the controllable contact forces
%
% Outputs:
% Hess_V_tot is the Hessian of the fucntion V 
%
function [ Hess_V_tot ] = H_V_tot( f_c, normals, mu, f_min, f_max , cf_dim, E  )
%
start_i = zeros(size(cf_dim)); % index of start of the fc_i vector
end_i = zeros(size(cf_dim)); % index of end of the fc_i vector
%
start_i(1) = 1 ;
for j = 2:length(cf_dim)
    start_i(j) = start_i(j-1)+cf_dim(j-1); 
end
for j = 1:length(cf_dim)
    end_i(j) = start_i(j) + cf_dim(j)-1 ;
end
%
Hess_V_tot = zeros(size(E,2));
%
for i = 1 : length(cf_dim)
    f_ci = f_c(start_i(i):end_i(i)) ; %, ...
    n_i  = normals(start_i(i):end_i(i)) ; %,...
    mu_i = mu(i) ;%
    f_min_i = f_min(i) ; %
    f_max_i = f_max(i) ; %, ...
    E_i   = E(start_i(i):end_i(i),:); %,
    % 
    Hess_V_tot = Hess_V_tot + H_V_i( f_ci, n_i, mu_i, f_min_i, f_max_i, E_i  ) ; %   V(i*2-1:2*i,1)
end
%
end
%
