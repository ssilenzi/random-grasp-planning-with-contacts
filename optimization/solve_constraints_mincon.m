% solve_constraints_particular_mincon
%
%   Finds the full minimizing solution which fulfills the contact
%   constraints from the particular solution
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

options1 = optimoptions('fmincon','TolFun',1e-30,'TolX',1e-30, ...
    'Algorithm','sqp', ...
    'MaxFunctionEvaluations',50000, ...
    'MaxIterations', 50000, ...
    'ConstraintTolerance', 1e-5);
[y_opt1,cost_opt1,exitflag1,output_info1,lambda1] = ...
    fmincon(hand_cost_fun, y0,[],[],[],[],[],[],[],options1) ;

options2 = optimoptions('fmincon','TolFun',1e-30,'TolX',1e-30, ...
    'Algorithm','interior-point', ...
    'MaxFunctionEvaluations',50000, ...
    'MaxIterations', 50000, ...
    'ConstraintTolerance', 1e-5);
[y_opt,cost_opt,exitflag,output_info,lambda] = ...
    fmincon(hand_cost_fun, y_opt1,[],[],[],[],[],[],[],options2) ;
    
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
