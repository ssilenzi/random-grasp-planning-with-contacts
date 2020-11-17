% V_optimal_global_mincon
%
%   Computes the minimum of the function V using the MATLAB supplied
%   global search using fmincon
%
%   Syntax: [fc_opt, y_opt, fval, exitflag, output_info, elapsed_time] = ...
%           V_optimal_mincon( f_c, normals, mu, f_min, f_max , cf_dim , E )
%
%   Inputs:
%   1) fp - vector collecting all the current contact forces (part. sol.)
%       Only linear forces are considered (f_ci is \in R^{3},
%       or \in R^{2} for planar problems).
%   2) normals - vector collecting the normals of the contact points
%       (n_i is \in R^{3}, or \in R^{2} for planar problems).
%   3) mu - vector collecting all the friction coefficient at the contacts
%   4) f_min - vector collecting all the min. values of normal forces
%   5) f_max - vector collecting all the max. values of normal forces
%   6) cf_dim \in R^{ (numb. of chains)} = the i-th element
%       specifies the number of the contact constraint on the
%       i-th contact point
%   7) E - basis of the controllable contact forces
%   8) y0 - initial guess for minimization
%
%   Outputs:
%   1) fc_opt - optimal contact force distribution (at the minimum of V)
%   2) y_opt - 'optimal coefficient vector' such that =>
%       => Fc_optimal = f_c0+E*y_opt (minimizing V)
%   3) V_opt - the value of the fucntion V at the minimum
%   4) V_0 - the value of the fucntion V at the the initial contact force
%       distribution
%   5) exitflag = Reason fminunc stopped
%   6) output_info  = Information about the optimization process
%   7) elapsed_time = time taken for the optimization process
%   8) sigma_leq = values of the sigma_ij that should be less or equal to zero
%	9) lambda = Lagrange multipliers at the solution point
%   10) grad = gradient at the solution point
%   11) hessian = Hessian matrix at the solution point
%

function [fc_opt, y_opt, cost_opt, cost_0, exitflag, output_info, ...
    elapsed_time, sigma_leq, lambda] = ...
    solve_constraints_mincon( fp, normals, mu, f_min, ...
    f_max , cf_dim , E, y0 )

time_0 = clock;
cost_0 = norm(fp + E*y0);

nvars = length(y0);    % Number of variables
% hand_cost_fun = @(y) 0; % ATTENTION! If 0 only the sigma are checked
hand_cost_fun = @(y) cost_fun(y,fp,E,normals,mu,f_min,f_max,cf_dim); 
hand_nonlcon = @(y) nonlcon(y,fp,E,normals,mu,f_min,f_max,cf_dim);

options = optimoptions('fmincon','TolFun',1e-30,'TolX',1e-30, ...
    'Algorithm','interior-point', ...
    'MaxFunctionEvaluations',30000, ...
    'MaxIterations', 90000, ...
    'ConstraintTolerance', 1e-5);
[y_opt,cost_opt,exitflag,output_info,lambda] = ...
    fmincon(hand_cost_fun, y0,[],[],[],[],[],[],[],options) ;

% [y_opt,cost_opt] = ga(hand_cost_fun,nvars,[],[],[],[],[],[], ...
%     hand_nonlcon);
    
    % Cost function to be minimized
    function cost = cost_fun(y,fp,E,normals,mu,f_min,f_max,cf_dim)
        f_c_loop = fp + E*y ;
        cost = V_tot(f_c_loop,normals,mu,f_min,f_max,cf_dim);
    end


    % Nonlinear contact constraints
    function [ sigma_leq, sigma_eq ] = nonlcon(y,fp,E,normals,mu,f_min,f_max,cf_dim)
        Delta = 0.00005;
        f_c_loop = fp + E*y ;
        sigma_leq = sigma_tot(f_c_loop,normals,mu,f_min,f_max,cf_dim);
        sigma_leq = sigma_leq - Delta * ones(size(sigma_leq));
        sigma_eq = [] ;
    end

elapsed_time = etime(clock, time_0) ;

% Optimal forces
fc_opt = fp + E*y_opt ;

% Constraints evaluation
sigma_leq = sigma_tot( fc_opt, normals, mu, f_min, f_max , cf_dim ) ;
end
%
