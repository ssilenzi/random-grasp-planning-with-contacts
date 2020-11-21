% solve_constraints_particular_mincon
%
%   Finds the particular K-norm minimizing solution which fulfills the
%   contact contstraints
%

function [fp_sol, cost_sol, cost0, exitflag, output, elapsed_time, ...
    sigma_leq] = solve_constraints_particular_mincon( we, fp0, G, K, ...
    normals, mu, f_min, f_max , cf_dim ,Delta )

time_0 = clock;
cost0 = nonlcon(fp0,normals,mu,f_min,f_max,cf_dim,Delta);

% hand_cost_fun = @(y) 0; % ATTENTION! If 0 only the sigma are checked
hand_cost_fun = @(fp) cost_fun(we,fp,G); 
hand_nonlcon1 = @(fp) nonlcon(fp,normals,mu,f_min,0.001*f_max,cf_dim,Delta);
hand_nonlcon2 = @(fp) nonlcon(fp,normals,mu,f_min,f_max,cf_dim,Delta);

% Getting a force inside the constraints (sigma = 0) to be used as initial
% guess for fmincon
options1 = optimoptions('fsolve', ...
    'MaxFunctionEvaluations',50000, ...
    'MaxIterations', 50000);
[fp1,cost1,exitflag1,output1] = fsolve(hand_nonlcon1,fp0,options1);

% Using the previous solution getting the K-pseudoinverse soultion
options2 = optimoptions('fmincon','TolFun',1e-30,'TolX',1e-30, ...
    'Algorithm','sqp', ...
    'MaxFunctionEvaluations',50000, ...
    'MaxIterations', 50000, ...
    'ConstraintTolerance', 1e-5);
[fp2,cost2,exitflag2,output2] = ...
    fmincon(hand_cost_fun, fp1,[],[],[],[],[],[],hand_nonlcon2,options2) ;
    
    % Cost function to be minimized - K norm
    function cost = cost_fun(we,fp,G)
        f_c_loop = fp ;
        cost = (1/2) * (we + G*f_c_loop).'*(we + G*f_c_loop);
    end


    % Nonlinear contact constraints - 
    function [ sigma_leq, sigma_eq ] = nonlcon(fp,normals,mu,f_min,f_max,cf_dim,Delta)
        f_c_loop = fp;
        sigma_leq = sigma_tot(f_c_loop,normals,mu,f_min,f_max,cf_dim);
        sigma_leq = sigma_leq - Delta * ones(size(sigma_leq));
        sigma_eq = [] ;
    end

elapsed_time = etime(clock, time_0) ;

% Result
fp_sol = fp0;
cost_sol = cost2;
exitflag = exitflag2;
output = output2;

% Constraints evaluation
sigma_leq = sigma_tot( fp_sol, normals, mu, f_min, f_max , cf_dim );
sigma_leq = sigma_leq - Delta * ones(size(sigma_leq));

end
%
