% H_V_i
%
%   Computes the Hessian of the function V (to be minimized) for one contact.
%   Friction cone, minimum and maximum normal force are considered
%
%   Syntax: [ Hess_V_i ] = H_V_i( f_ci, n_i, mu_i, f_min_i, f_max_i, E_i  )
%
%   Inputs:
%   f_ci is the current (to be evaluated) contact force on the i-th contact
%       Only linear forces are considered (f_ci is \in R^{3}, or \in R^{2} for planar problems).
%   n_i  is the direction of the normal to the contact surface on the i-th contact
%       (n_i is \in R^{3}, or \in R^{2} for planar problems).
%   mu_i \in R+, is the friction coefficient at the the i-th contact
%   f_min_i \in R;  magnitude of the minimum normal force admitted
%   f_max_i \in R;  magnitude of the maximum normal force admitted
%   E_i = basis of the controllable contact forces
%
% Outputs:
% Hess_V_i is the Hessian matrix of the function V relative to the i-th contact
%
function [ Hess_V_i ] = H_V_i( f_ci, n_i, mu_i, f_min_i, f_max_i, E_i  )
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
%
% for j = 1:length(sig_vect)
%     sig_vect(j) = alpha(j)*norm(f_ci) + beta(j)*f_ci'*n_i + gamma(j)  ;
% end
sig_vect = sigma_i( f_ci, n_i, mu_i, f_min_i, f_max_i  );
% disp('Sig vect in H_V_i is '); disp(sig_vect);
%
epsilon = eps^(1/8) ; %
a = (3/2)*(1/epsilon^4) ;
b = 4*(1/epsilon^3) ;
%c = 3/(epsilon^2) ;  %1/(15*epsilon^2); % -5/epsilon^2; % %
%
Hess_V_i = zeros(size(E_i,2)) ;
for j = 1: length(sig_vect)
%     d_sig_y =  ( alpha(j)*((f_ci)'*(f_ci))^(-1/2) * ((E_i')*f_ci ) +...
%                beta(j)*E_i'*n_i  ) ; 
%     alpha(j)*(f_ci'*f_ci)^(-1/2) * (E_i'*f_ci ) +...
%                     beta(j)*E_i'*n_i   ;
    d2_sig_y2 =  alpha(j) * E_i' * ...
        ( eye(size(f_ci*f_ci')) -  (f_ci*f_ci')/( norm(f_ci) ) ) *  E_i ;
    disp('norm(f_ci) in H_V_i is '); disp(norm(f_ci));
    %
    if ( sig_vect(j) <=-epsilon )
        Hess_V_i = Hess_V_i  -( sig_vect(j) )^(-3) *d2_sig_y2 + ...
                      3*( sig_vect(j) )^(-4) *(d2_sig_y2*d2_sig_y2') ;
    else
        Hess_V_i = Hess_V_i + (2*a*sig_vect(j) + b)*d2_sig_y2   +...
                      2*a *(d2_sig_y2*d2_sig_y2')   ;
    end
    
%     disp(Hess_V_i);
end
%
end