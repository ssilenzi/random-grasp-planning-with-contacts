% V_contact
%
%   Computes the term of the fucntion V (to be minimized) for a contact.
%   Friction cone, minimum and maximum normal force are considered
%
%   Syntax:  [ V_i ] = V_contact( f_ci, n_i, mu_i, f_min_i, f_max_i  )
%
%   Inputs:
%   f_ci is the current (to be evaluated) contact force on the i-th contact
%       Only linear forces are considered (f_ci is \in R^{3}, or \in R^{2} for planar problems).
%   n_i  is the direction of the normal to the contact surface on the i-th contact
%       (n_i is \in R^{3}, or \in R^{2} for planar problems).
%   mu_i \in R+, is the friction coefficient at the the i-th contact
%   f_min_i \in R;  magnitude of the minimum normal force admitted
%   f_max_i \in R;  magnitude of the maximum normal force admitted
%
% Outputs:
% V_i is term of the fucntion V relative to the i-th contact
%
function [ sigma_vec ] = sigma_tot( f_c, normals, mu, f_min, f_max , cf_dim )
%

% start_i = zeros(size(cf_dim)); % index of start of the fc_i vector
% end_i = zeros(size(cf_dim)); % index of end of the fc_i vector
% %
% start_i(1) = 1 ;
% for j = 2:length(cf_dim)
%     start_i(j) = start_i(j-1)+cf_dim(j-1);
% end
% for j = 1:length(cf_dim)
%     end_i(j) = start_i(j) + cf_dim(j)-1 ;
% end
% V = 0;

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