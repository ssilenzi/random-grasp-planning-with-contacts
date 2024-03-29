% solve_constraints_particular_mincon
%
%   Finds the full minimizing solution which fulfills the contact
%   constraints from the particular solution
%

function [fc_opt, y_opt, cost_opt, cost0, exitflag, output, ...
    elapsed_time, sigma_leq] = ...
    solve_constraints_full_mincon_correct( fp, E, y0, normals, mu, f_min, ...
    f_max , m_min, m_max, cf_dim , Delta )

max_eval = 1e20;
max_iter = 1e20;
constr_tol = 1e-20;
step_tol = 1e-20;

time_0 = clock;
cost0 = nonlcon(fp,E,y0,normals,mu,f_min,f_max,m_min,m_max,cf_dim,Delta);

hand_cost_fun = @(y) cost_fun(fp,E,y,normals,mu,f_min,f_max,m_min,m_max,cf_dim);
hand_nonlcon = @(y) nonlcon(fp,E,y,normals,mu,f_min,f_max,m_min,m_max,cf_dim,Delta);

% Using the previous solution getting the K-pseudoinverse soultion
options2 = optimoptions('fmincon','TolFun',1e-30,'TolX',1e-30, ...
    'Algorithm','interior-point', ...
    'MaxFunctionEvaluations',max_eval, ...
    'MaxIterations', max_iter, ...
    'StepTolerance', step_tol, ...
    'TolX', step_tol, ...
    'ConstraintTolerance', constr_tol, 'Display', 'off');
[y2,cost2,exitflag2,output2] = ...
    fmincon(hand_cost_fun, y0,[],[],[],[],[],[],[],options2);
    
    % Cost function to be minimized - forces equilibria
    function cost = cost_fun(fp,E,y,normals,mu,f_min,f_max,m_min,m_max,cf_dim)
        f_c_loop = fp + E*y;
        V_f = V_tot(f_c_loop,normals,mu,f_min,f_max,m_min,m_max,cf_dim);
        cost_vec = V_f.';
        cost = norm(cost_vec);
    end


    % Nonlinear contact constraints
    function [ sigma_leq, sigma_eq ] = nonlcon(fp,E,y,normals,mu,f_min,f_max,m_min,m_max,cf_dim,Delta)
        f_c_loop = fp + E*y;
        sigma_leq = sigma_tot(f_c_loop,normals,mu,f_min,f_max,m_min,m_max,cf_dim);
        sigma_leq = sigma_leq - Delta * ones(size(sigma_leq));
        sigma_eq = [] ;
    end

elapsed_time = etime(clock, time_0) ;

% Result
y_opt = y2;
fc_opt = fp + E*y_opt;
cost_opt = cost2;
exitflag = exitflag2;
output = output2;

% Constraints evaluation
sigma_leq = sigma_tot( fc_opt, normals, mu, f_min, f_max ,m_min,m_max, cf_dim );
sigma_leq = sigma_leq - Delta * ones(size(sigma_leq));

end
%
