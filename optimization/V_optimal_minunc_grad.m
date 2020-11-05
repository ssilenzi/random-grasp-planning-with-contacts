% V_optimal_minunc_grad
%
%   Computes the minimum of the fucntion V using the MATLAB supplied
%   fminunc,  using also the formula for the gradient of the function
%
%   Syntax: [fc_opt, y_opt, fval, exitflag, output_info, elapsed_time] = ...
%               = V_optimal_minunc( f_c, normals, mu, f_min, f_max , cf_dim )
%
%   Inputs:
%   f_c0   is a vector collecting all the current contact forces (before the optimization)
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
% fc_opt = optimal contact force distribution (corresponding to the minimum of V)
% y_opt = 'optimal coefficient vector' such that =>
%            => Fc_optimal = f_c0+E*y_opt (minimizing V)
% fval  = the value of the fucntion V at the minimum
% exitflag = Reason fminunc stopped
% output_info  = Information about the optimization process
% elapsed_time = time taken for the optimization process
% sigma_leq = values of the sigma_ij that should be less or equal to zero
% grad = gradient at the solution point
% hessian = Hessian matrix at the solution point
%
function [fc_opt, y_opt, V_opt, V_0, exitflag, output_info, elapsed_time ,  sigma_leq, grad,hessian] = V_optimal_minunc_grad( f_c0, normals, mu, f_min, f_max , cf_dim , E )
%
time_0 = clock;
y0 = zeros(size(E,2),1) ;
V_0 = V_tot( f_c0, normals, mu, f_min, f_max , cf_dim ) ;
%
%options = optimoptions(@fminunc,'Algorithm','quasi-newton');
options = optimoptions(@fminunc,'GradObj','on', ... %'Algorithm','trust-region',
                    'TolFun',1e-20,'TolX',1e-20, 'MaxIter',1000,'MaxFunEvals',1000);
%'TolFun',1e-20,'TolX',1e-20, 'MaxIter',1000,'MaxFunEvals',1000);
                   %'TolX',1E-30, 'TolFun', 1E-30, 'MaxIter', 1000, 'MaxFunEvals',1500); %,'SpecifyObjectiveGradient',true);
[y_opt,V_opt,exitflag,output_info,grad,hessian] = fminunc(@V_minunc, y0, options);
    function [V_min, V_grad_y ] = V_minunc(y)
        f_c_loop = f_c0 + E*y ;
        V_min = V_tot( f_c_loop, normals, mu, f_min, f_max , cf_dim ) ;
        V_grad_y = D_V_tot( f_c_loop, normals, mu, f_min, f_max , cf_dim, E  ) ;
        %V_grad_y = V_grad_sig*(2*a*E'*(f_c0+E*y)+b*E'*normals );
    end
elapsed_time = etime(clock, time_0) ;
fc_opt = f_c0 + E*y_opt ;
% Constraint evaluation
sigma_leq = sigma_tot( fc_opt, normals, mu, f_min, f_max , cf_dim ) ;
%
end
%
