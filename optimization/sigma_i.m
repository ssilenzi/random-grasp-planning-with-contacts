% Sigma Tot
%
%   Computes the ith constraints (to be minimized) for a contact.
%   Friction cone, minimum and maximum normal force are considered
%
%   Syntax:  [ Sigma_i ] = Sigma_tot( f_ci, n_i, mu_i, f_min_i, f_max_i )
%

function [ sigma_i ] = sigma_i( f_ci, n_i, mu_i, f_min_i, f_max_i )
%
sig_vect = zeros(3,1) ;
alpha = zeros(3,1) ;  % friction constraint
beta  = zeros(3,1) ;  % minimum contact force constraint
gamma = zeros(3,1) ;  % maximum contact force constraint
%
alpha(1) = 1/sqrt(1+mu_i^2 ) ; % friction constraint
%
beta(1) = -1 ; % friction constraint
beta(2) = -1 ; % minimum contact force constraint
beta(3) =  1 ; % maximum contact force constraint
%
gamma(2) =  f_min_i ; % minimum contact force constraint
gamma(3) = -f_max_i ; % maximum contact force constraint

if length(f_ci) == 1
    sig_vect(1) = beta(1)*f_ci'*n_i;
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