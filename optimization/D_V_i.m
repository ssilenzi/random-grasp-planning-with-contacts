% D_V_i
%
%   Computes the derivative of the term of the function V (to be minimized) for a contact.
%   Friction cone, minimum and maximum normal force are considered
%
%   Syntax:  [ V_grad_i ] = D_V_i( f_ci, n_i, mu_i, f_min_i, f_max_i, E_i )
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
% V_grad_i is term of the derivative of the function V relative to the i-th contact
%
function [ V_grad_i ] = D_V_i( f_ci, n_i, mu_i, f_min_i, f_max_i, E_i  )
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
for j = 1:length(sig_vect)
    sig_vect(j) = alpha(j)*norm(f_ci) + beta(j)*f_ci'*n_i + gamma(j)  ;
end
%
epsilon = eps^(1/8) ; % 
a = (3/2)*(1/epsilon^4) ;
b = 4*(1/epsilon^3) ;
%c = 3/(epsilon^2) ;  %1/(15*epsilon^2); % -5/epsilon^2; % %
%

% disp(' alpha is '); disp(alpha);
% disp(' beta is '); disp(beta);
% disp(' gamma is '); disp(gamma);
% disp(' n_i is '); disp(n_i);
% disp(' mu_i is '); disp(mu_i);
% disp(' f_min_i is '); disp(f_min_i);
% disp(' f_max_i is '); disp(f_max_i);
% disp(' E_i is '); disp(E_i);
% disp(' sig_vect is '); disp(sig_vect);

% disp('Sig vect in D_V_i is '); disp(sig_vect);

V_grad_ij= zeros( size(E_i,2) ,length(sig_vect)) ;
for j = 1: length(sig_vect)
    if ( sig_vect(j) <=-epsilon )
        V_grad_ij(:,j) = -( sig_vect(j) )^(-3) * ...
             ( alpha(j) * ((E_i')*f_ci ) + ...
               beta(j) * E_i'*n_i  ) ;
    else
        V_grad_ij(:,j) = (2*a*sig_vect(j) + b) * ...
             ( alpha(j) * ((E_i')*f_ci) + ...  %  
               beta(j) * E_i'*n_i )  ;
    end
end
V_grad_i = sum(V_grad_ij')';  
%
end