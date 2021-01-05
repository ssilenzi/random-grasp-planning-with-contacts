% Sigma Tot
%
%   Computes the ith constraints (to be minimized) for a contact.
%   Friction cone, minimum and maximum normal force are considered
%
%   Syntax:  [ Sigma_i ] = Sigma_tot( f_ci, n_i, mu_i, f_min_i, f_max_i )
%

function [ sigma_i ] = sigma_i( f_ci, n_i, mu_i, f_min_i, f_max_i, m_min_i, m_max_i )
%
sig_force_vect = zeros(3,1) ;
sig_mom_vect = zeros(2,1) ;
alpha = zeros(5,1) ;  % friction constraint
beta  = zeros(5,1) ;  % minimum contact force constraint
gamma = zeros(5,1) ;  % maximum contact force constraint
%
alpha(1) = 1/sqrt(1+mu_i^2 ) ; % friction constraint
%
beta(1) = -1 ; % friction constraint
beta(2) = -1 ; % minimum contact force constraint
beta(3) =  1 ; % maximum contact force constraint
beta(4) = -1 ; % minimum contact force constraint
beta(5) =  1 ; % maximum contact force constraint
%
gamma(2) =  f_min_i ; % minimum contact force constraint
gamma(3) = -f_max_i ; % maximum contact force constraint
gamma(4) =  m_min_i ; % minimum contact force constraint
gamma(5) = -m_max_i ; % maximum contact force constraint

if length(f_ci) == 1 % Sliding
    sig_force_vect(1) = beta(1)*f_ci;
    sig_force_vect(2) = beta(2)*f_ci + gamma(2);
    sig_force_vect(3) = beta(3)*f_ci + gamma(3);
else % Maintained hard finger
    force = f_ci(1:3);
    for j=1:length(sig_force_vect) % The force part is the same
        sig_force_vect(j) = alpha(j)*norm(force) + beta(j)*force'*n_i + gamma(j);
    end
    
    if length(f_ci) == 4 % Maintained soft finger
        % Moment part has only max and min
        moment = f_ci(4);
        sig_mom_vect(1) = beta(4)*norm(moment) + gamma(4);
        sig_mom_vect(2) = beta(5)*norm(moment) + gamma(5);
    end
end
sigma_i = sig_force_vect ;
if length(f_ci) == 4
    sigma_i = [sigma_i; sig_mom_vect];
end
%
end