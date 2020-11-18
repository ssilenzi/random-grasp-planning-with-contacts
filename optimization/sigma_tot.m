% Sigma Tot
%
%   Computes the constraints (to be minimized) for a contact.
%   Friction cone, minimum and maximum normal force are considered
%
%   Syntax:  [ Sigma_tot ] = Sigma_tot( f_ci, n_i, mu_i, f_min_i, f_max_i, cf_dim )
%

function [ sigma_vec ] = sigma_tot( f_c, normals, mu, f_min, f_max , cf_dim )

ind = 1;
for i = 1: length(cf_dim)
    indf = ind+cf_dim(i)-1;
    sigma_temp( 3*i-2: 3*i ) = sigma_i( f_c(ind: indf), ...
        normals(ind: indf),...
        mu(i), f_min(i), f_max(i) ) ;
    ind = indf+1;
end
sigma_vec = sigma_temp ;
%
end