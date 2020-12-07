% solve_constraints_particular_mincon
%
%   Finds the particular minimizing solution which fulfills the
%   contact contstraints
%

function [fp_sol, cost_sol, cost0, exitflag, output, elapsed_time, ...
    sigma_leq] = solve_constraints_particular_mincon( we, fp0, G, K, ...
    normals, mu, f_min, f_max , cf_dim , Delta )

max_eval = 3000;
max_iter = 1000;

time_0 = clock;
cost0 = nonlcon(fp0,normals,mu,f_min,f_max,cf_dim,Delta);

% hand_cost_fun = @(y) 0; % ATTENTION! If 0 only the sigma are checked
hand_zero_fun = @(fp) 0;
hand_cost_fun = @(fp) cost_fun(fp,normals,mu,f_min,f_max,cf_dim);
hand_nonlcon = @(fp) nonlcon(fp,normals,mu,f_min,f_max,cf_dim,Delta);

% Getting a force inside the constraints (sigma = 0) to be used as initial
% guess for the second fmincon

% options1 = optimoptions('fmincon','TolFun',1e-30,'TolX',1e-30, ...
%     'Algorithm','sqp', ...
%     'MaxFunctionEvaluations',100000, ...
%     'MaxIterations', 50000, ...
%     'ConstraintTolerance', 1e-5);
% [fp1,cost1,exitflag1,output1] = ...
%     fmincon(hand_zero_fun,fp0,[],[],[],[],[],[],hand_nonlcon1,options1) ;

options1 = optimoptions('fmincon','TolFun',1e-30,'TolX',1e-30, ...
    'Algorithm','interior-point', ...
    'MaxFunctionEvaluations',max_eval, ...
    'MaxIterations', max_iter, ...
    'ConstraintTolerance', 1e-5, 'Display', 'off');
[fp1,cost1,exitflag1,output1] = ...
    fmincon(hand_cost_fun,fp0,[],[],[],[],[],[],[],options1);

% Using the previous solution getting the K-pseudoinverse soultion
options2 = optimoptions('fmincon','TolFun',1e-30,'TolX',1e-30, ...
    'Algorithm','interior-point', ...
    'MaxFunctionEvaluations',max_eval, ...
    'MaxIterations', max_iter, ...
    'ConstraintTolerance', 1e-5, 'Display', 'off');
[fp2,cost2,exitflag2,output2] = ...
    fmincon(hand_zero_fun, fp1,[],[],-G,we,[],[],hand_nonlcon,options2);

if rank(G) < 6
    disp('G is not full rank in solve_constraints. The rank is'); disp(rank(G));
    disp('and G is '); disp(G);
end
    
    % Cost function to be minimized - forces equilibria
    function cost = cost_fun(fp,normals,mu,f_min,f_max,cf_dim)
        f_c_loop = fp ;
        V_f = V_tot(f_c_loop,normals,mu,f_min,f_max,cf_dim);
        cost_vec = V_f.';
        cost = norm(cost_vec);
    end


    % Nonlinear contact constraints
    function [ sigma_leq, sigma_eq ] = nonlcon(fp,normals,mu,f_min,f_max,cf_dim,Delta)
        f_c_loop = fp;
        sigma_leq = sigma_tot(f_c_loop,normals,mu,f_min,f_max,cf_dim);
        sigma_leq = sigma_leq - Delta * ones(size(sigma_leq));
        sigma_eq = [] ;
    end

elapsed_time = etime(clock, time_0) ;

% Result
fp_sol = fp2;
cost_sol = cost2;
exitflag = exitflag2;
output = output2;

% Constraints evaluation
sigma_leq = sigma_tot( fp_sol, normals, mu, f_min, f_max , cf_dim );
sigma_leq = sigma_leq - Delta * ones(size(sigma_leq));

end
%
