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

function [fc_opt, y_opt, V_opt, V_0, exitflag, output_info, ...
    elapsed_time, sigma_leq, lambda, grad, hessian ] = ...
    V_optimal_global_mincon( fp, normals, mu, f_min, ...
    f_max , cf_dim , E, y0 )

time_0 = clock;
V_0 = V_tot( fp, normals, mu, f_min, f_max , cf_dim ) ;

% options = optimoptions(@fmincon,'Algorithm','sqp',...
%     'TolFun',1e-30,'TolX',1e-30, 'MaxIter',1000000,'MaxFunEvals',5000000);
% [y_opt,V_opt,exitflag,output_info,lambda,grad,hessian] = fmincon(@V_mincon, y0,[],[],[],[],[],[],@nonlcon,options) ;
options = optimoptions('fmincon','TolFun',1e-30,'TolX',1e-30, ...
    'Algorithm','interior-point');
%     'SpecifyObjectiveGradient',true,'HessianFcn',@Hess_Optim_mincon);
[y_opt,V_opt,exitflag,output_info,lambda,grad,hessian] = ...
    fmincon(@Optim_mincon, y0,[],[],[],[],[],[],[],options) ;
    
    % Cost function to be minimized
    function [V_min, D_V_min, H_V_min] = Optim_mincon(y)
        f_c_loop = fp + E*y ;
        V_min = V_tot( f_c_loop, normals, mu, f_min, f_max , cf_dim ) ;
        if nargout > 1              % Gradient
            D_V_min = D_V_tot( f_c_loop, normals, mu, f_min, ...
                f_max , cf_dim, E ) ;
            if nargout > 2          % Hessian
                H_V_min = H_V_tot( f_c_loop, normals, mu, f_min, ...
                    f_max , cf_dim, E ) ;
            end
        end
    end

    % Hessian for interior-point
    function H_V_min_int = Hess_Optim_mincon(y, lambda)
        f_c_loop = fp + E*y ;
        % Hessian of objective
        H_V_min = H_V_tot( f_c_loop, normals, mu, f_min, ...
            f_max , cf_dim, E ) ;
        % Hessian final (no nonlin constr.)
        H_V_min_int = H_V_min;
    end

%     % Nonlinear contact constraints (NOT USED!)
%     function [sigma_leq,sigma_eq ] = nonlcon(y)
%         f_c_loop = fp + E*y ;
%         sigma_leq = sigma_tot( f_c_loop, normals, mu, f_min, f_max , cf_dim ) ;
%         % sigma_leq = sigma_leq ; % + epsilon *ones(size(sigma_leq,1),1) ;
%         sigma_eq = [] ;
%     end

elapsed_time = etime(clock, time_0) ;

% Optimal forces
fc_opt = fp + E*y_opt ;

% Constraints evaluation
sigma_leq = sigma_tot( fc_opt, normals, mu, f_min, f_max , cf_dim ) ;
end
%
