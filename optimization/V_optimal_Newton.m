% V_optimal_Newton
%
%   Computes the minimum of  fucntion V using the Newton method
%      y_(k+1) = y(k) - (H)^{-1}*grad(V)
%
%   Syntax: [fc_opt, y_opt, V_opt, V_0, exit_tests , elapsed_time, sigma_leq, grad_opt, Hessian_opt] = ...
%                           V_optimal_Newton( f_c, normals, mu, f_min, f_max , cf_dim, E , max_iter (opt.), norm_grad (opt.) ) 
%
%   Inputs:
%   f_c0     is a vector collecting all the current contact forces
%       Only linear forces are considered (f_ci is \in R^{3}, or \in R^{2} for planar problems). 
%   normals is a vector collecting di directions of all the normals a the contact points
%       (n_i is \in R^{3}, or \in R^{2} for planar problems). 
%   mu    is the vector collecting all the friction coefficient at the contacts
%   f_min is the vector collecting all the minimum normal forces admitted at the contacts
%   f_max is the vector collecting all the maximum normal forces admitted at the contacts
%   cf_dim \in R^{ (numb. of chains)} = the i-th element specifies the number of
%          the contact constraint on the i-th contact point
%   E = basis of the controllable contact forces
%   max_iter (optional) = maximum number of iterations
%   norm_grad (optional)= value of the norm of gradient accettable to quit
%
% Outputs:
% fc_opt = optimal contact force distribution (corresponding to the minimum of V)
% y_opt = 'optimal coefficient vector' such that =>
%            => Fc_optimal = f_c0+E*y_opt (minimizing V)
% V_opt  = the value of the fucntion V at the minimum
% V_0    = the value of the fucntion V at the the initial contact force
%          distribution
% exit_tests = structure => .max_iter  = results of the verification of the maximum number of iterations ;
%                   .norm_grad  = results of the verification of the norm of the gradient ; 
%                   .iter = number of iterations eecuted
% elapsed_time = time taken for the optimization process
% sigma_leq = values of the sigma_ij that should be less or equal to zero
% grad_opt = gradient at the solution point
% Hessian_opt = Hessian matrix at the solution point
%
function [fc_opt, y_opt, V_opt, V_0, exit_tests , elapsed_time, sigma_leq, grad_opt, Hessian_opt, V_vect] = V_optimal_Newton( f_c, normals, mu, f_min, f_max , cf_dim, E , varargin) 
%
time_0 = clock;
y0 = zeros(size(E,2),1) ;
%
max_iter          = 100000 ;
norm_grad         = 0.0001 ;
% 
% if(size(varargin,1)~=0)
%     if(size(varargin,2)==1)
%     max_iter  = varargin{1} ;    
%     %
%     elseif(size(varargin,2)==2)
%     max_iter  = varargin{1}  ;
%     norm_grad = varargin{2}  ;
%     %
%     end
% end


if( max_iter<1 )
    max_iter = 1 ;
elseif(max_iter>1E8)
    max_iter = 1E8;
end
if( norm_grad<1E-30 )
    norm_grad=1E-30  ;
end
%
f_c0 = f_c - E*y0 ;
%
iter = 1;
y_old = y0 ;
V_0 = V_tot( f_c, normals, mu, f_min, f_max , cf_dim ) ;
V_old = V_0 ;
y_new = y_old ;
V_new = V_old ;
% grad_V    = D_V_y_( f_c, normals, mu, f_min, f_max , cf_dim, E  ); % D_V_tot( f_c, normals, mu, f_min, f_max , cf_dim, E  ) ;
% Hessian_V = H_V_tot_( f_c, normals, mu, f_min, f_max , cf_dim, E  );  % H_V_tot

grad_V    = D_V_tot( f_c, normals, mu, f_min, f_max , cf_dim, E  ); % D_V_tot( f_c, normals, mu, f_min, f_max , cf_dim, E  ) ;
Hessian_V = H_V_tot( f_c, normals, mu, f_min, f_max , cf_dim, E  );
%
% if tests are true then optimize
% disp('iter '); disp(iter);
% disp('max_iter '); disp(max_iter);
% disp('norm(grad_V) is '); disp(norm(grad_V));
% disp('norm_grad '); disp(norm_grad);
test_1 = (iter<max_iter) ;
test_2 = (norm(grad_V)> norm_grad ) ;
test = test_1&&test_2 ;
%
V_vect = [] ;
while( test )
%         disp('Entered while loop!!');
       y_new = y_old  +  - Hessian_V\grad_V  ; 
       fc_new = f_c0 + E*y_new ;
       V_new     = V_tot( fc_new , normals, mu, f_min, f_max , cf_dim ) ;
       grad_V    = D_V_tot( fc_new, normals, mu, f_min, f_max , cf_dim, E  ) ;
       Hessian_V = H_V_tot( fc_new, normals, mu, f_min, f_max , cf_dim, E  );
       %
       V_vect(iter) = V_new ;
              iter = iter + 1 ;     

       test_1 = (iter<max_iter) ;
       test_2 = (norm(grad_V)> norm_grad) ;
       test = test_1&&test_2 ;
end
elapsed_time = etime(clock, time_0) ;
%
exit_tests.max_iter  = test_1 ;
exit_tests.norm_grad = test_2 ;
exit_tests.iter  = iter ;
%
y_opt = y_new ;
fc_opt =   fc_new ;
V_opt  =   V_new  ;
grad_opt = grad_V ;
Hessian_opt = Hessian_V ;
% Constraint evaluation
sigma_leq = sigma_tot( fc_opt, normals, mu, f_min, f_max , cf_dim ) ;
% 
end
%
