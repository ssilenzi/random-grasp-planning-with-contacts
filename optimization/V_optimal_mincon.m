% V_optimal_mincon
%
%   Computes the minimum of the function V using the MATLAB supplied
%   fmincon
%
%   Syntax: [fc_opt, y_opt, fval, exitflag, output_info, elapsed_time] = ...
%           V_optimal_mincon( f_c, normals, mu, f_min, f_max , cf_dim , E )
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
%
% Outputs:
% fc_opt = optimal contact force distribution (corresponding to the minimum of V)
% y_opt = 'optimal coefficient vector' such that =>
%            => Fc_optimal = f_c0+E*y_opt (minimizing V)
% V_opt  = the value of the fucntion V at the minimum
% V_0    = the value of the fucntion V at the the initial contact force
%          distribution
% exitflag = Reason fminunc stopped
% output_info  = Information about the optimization process
% elapsed_time = time taken for the optimization process
% sigma_leq = values of the sigma_ij that should be less or equal to zero
% lambda = Lagrange multipliers at the solution point
% grad = gradient at the solution point
% hessian = Hessian matrix at the solution point
%
function [fc_opt, y_opt, V_opt, V_0, exitflag, output_info, elapsed_time, sigma_leq, lambda,grad,hessian ] = V_optimal_mincon( fc_0, normals, mu, f_min, f_max , cf_dim , E )
%
time_0 = clock;
y0 = rand(size(E,2),1) ;
V_0 = V_tot( fc_0, normals, mu, f_min, f_max , cf_dim ) ;
% epsilon = eps^(1/8) ;
%
options = optimoptions(@fmincon,'Algorithm','sqp',...
    'TolFun',1e-30,'TolX',1e-30, 'MaxIter',1000,'MaxFunEvals',5000);
[y_opt,V_opt,exitflag,output_info, lambda,grad,hessian] = fmincon(@V_mincon, y0,[],[],[],[],[],[],@nonlcon,options) ;
    function V_min = V_mincon(y)
        f_c_loop = fc_0 + E*y ;
        V_min = V_tot( f_c_loop, normals, mu, f_min, f_max , cf_dim ) ;
       % V_grad_y = D_V_y( f_c_loop, normals, mu, f_min, f_max , cf_dim, E  ) ;
    end
    function [sigma_leq, sigma_eq ] = nonlcon(y)
        f_c_loop = fc_0 + E*y ;
        sigma_leq = sigma_tot( f_c_loop, normals, mu, f_min, f_max , cf_dim ) ;
        % sigma_leq = sigma_leq ; % + epsilon *ones(size(sigma_leq,1),1) ;
        sigma_eq = [] ;
    end
elapsed_time = etime(clock, time_0) ;
fc_opt = fc_0 + E*y_opt ;
%
% Constraint evaluation
sigma_leq = sigma_tot( fc_opt, normals, mu, f_min, f_max , cf_dim ) ;
end
%
