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
function [ sigma_i ] = sigma_i( f_ci, n_i, mu_i, f_min_i, f_max_i  )
%
sig_vect = zeros(3,1) ;
alpha = zeros(3,1) ;  % friction constraint
beta  = zeros(3,1) ;  % minimum contact force constraint
gamma = zeros(3,1) ;  % maximum contact force constraint
%
alpha(1) = 1/sqrt(1+mu_i^2 ) ; % friction constraint
% % alpha(2) = 0 ; % minimum contact force constraint
% % alpha(3) = 0 ; % maximum contact force constraint
%
beta(1) = -1 ; % friction constraint
beta(2) = -1 ; % minimum contact force constraint
beta(3) =  1 ; % maximum contact force constraint
%
% % gamma(1) =  0  ; % friction constraint
gamma(2) =  f_min_i ; % minimum contact force constraint
gamma(3) = -f_max_i ; % maximum contact force constraint
if length(f_ci) == 1
    sig_vect(1) = -f_ci;
    sig_vect(2) = beta(2)*f_ci + gamma(2);
    sig_vect(3) = beta(3)*f_ci + gamma(3);
else
    for j=1:length(sig_vect)
        sig_vect(j) = alpha(j)*norm(f_ci) + beta(j)*f_ci'*n_i + gamma(j);
    end
end
sigma_i = sig_vect ;
%
end