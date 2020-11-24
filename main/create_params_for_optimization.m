function [normals,mu_vect,f_min_vect,f_max_vect,cf_dim_tot] = ...
    create_params_for_optimization(Cp_h,Cn_h,Cp_e_prime,Cn_e_prime, ...
    c_types, mu_h_val, mu_e_val, f_min_h, f_max_h, f_min_e, f_max_e)
% CREATE PARAMS FOR OPTIMIZATION 
%   Inputs:
%   Cp_h        - hand contact positions (rows)
%   Cn_h        - hand contact normals (rows)
%   Cp_e_prime  - env. contact positions (rows) without the detached
%   Cn_e_prime  - env. contact normals (rows) without the detached
%   c_types     - vec. showing the types of contacts
%   other vals  - self describing...
%   Outputs:
%   normals 	- vec. collecting di directions of all the normals a the 
%               contact points(n_i is \in R^{3}, or \in R^{2} for planar problems). 
%   mu_vect    	- vec. collecting all the friction coefficient at the contacts
%   f_min_vect	- vec. collecting all the minimum normal forces admitted at 
%               the contacts
%   f_max_vect  - vec. collecting all the maximum normal forces allowed 
%               at the contacts
%   cf_dim_tot  - vec. with dimensions of the all the cont. forces to be 
%               optimized (hand + env). In contact_type_analysis a similar
%               vector contains only the env values

% Normals for the optimization function
normals = [];
cf_dim_tot = [];
num_cp = 0;     % Total no. of contacts
for i=1:size([Cp_h;Cp_e_prime],1)
    if i <= size(Cp_h,1)
        normals =  [normals; Cn_h(i,:).'];
        cf_dim_tot = [cf_dim_tot, 3];
    else
        if c_types(i-size(Cp_h,1)) == 1
            cf_dim_tot = [cf_dim_tot, 3];
        elseif c_types(i-size(Cp_h,1)) == 3
            cf_dim_tot = [cf_dim_tot, 1];
        end
           normals = [normals; Cn_e_prime(i -size(Cp_h,1),:).'];
    end
    num_cp = num_cp+1;
end

% Other force constraints for optimization
mu_hand = mu_h_val;
mu_env = mu_e_val;
mu_vect = [ones(1,size(Cp_h,1))*mu_hand ones(1,size(Cp_e_prime,1))*mu_env];
f_min_vect = f_min_e*ones(1,num_cp); f_min_vect(1:2) = f_min_h*ones(1,2);
f_max_vect = f_max_e*ones(1,num_cp); f_max_vect(1:2) = f_max_h*ones(1,2);

end

