function [wrench_opt_tot,Cf,Cp_viol,Cf_viol] = ...
    post_process_forces(Cp_h,Cn_h,Cp_e_prime,Cn_e_prime,d_pose, ...
    fc_opt,c_types,cf_dim_tot,sigma_leq,Delta,mu_env)

% POST PROCESS FORCES - This function is used to transform the optimized
% forces vector to the total vector (i.e. the normal component of the
% sliding forces are used to get also the tangential component and put
% together). Then, also the positions and entity of the contact forces that
% violate the constraints are extracted, for later plotting.
%   Inputs:
%   Cp_h        - hand contact positions (rows)
%   Cn_h        - hand contact normals (rows)
%   Cp_e_prime  - env. contact positions (rows) without the detached
%   Cn_e_prime  - env. contact normals (rows) without the detached
%   d_pose      - variation of object pose
%   c_types     - vec. showing the types of contacts
%   cf_dim_tot 	- vec. with dimensions of the optimized cont. forces
%   sigma_leq   - vec. with sigma constraint values
%   Delta       - the small positive margin used for fmincon optimization
%   other vals  - self describing...
%   Outputs:
%   fc_opt_tot  - total cont. forces vector
%   Cf          - matrix withforces put in rows (for plotting)
%   Cp_viol     - positions of the violated cont. forces
%   Cf_viol    	- entity of the violated cont. forces

% Now the normal comp. of the sliding forces shall be trasformed so as to
% get the whole forces including the tangential component too.

Cp_tot = [Cp_h; Cp_e_prime];
Cn_tot = [Cn_h; Cn_e_prime];
wrench_opt_tot = [];

ind = 1; % Index for the forces

soft_fing = cf_dim_tot(1) == 4;
if soft_fing
    indm = 3*length(find(cf_dim_tot == 4));
    num_hand_conts = length(find(cf_dim_tot == 4));
end

for i = 1: size(Cn_tot,1)
    
    % Getting the dimension of the present forces part (1 if sliding, 3
    % otherwise)
    force_dim = cf_dim_tot(i);
    if force_dim == 4
        force_dim = 3;
    end
    
    % Updating the final indexes of wrenches
    indf = ind+force_dim-1;
    
    wrench_tmp = [];
    hand_ind = size(Cn_h,1);
    if i <= hand_ind
        wrench_tmp = fc_opt(ind:indf,:);
    else
        if c_types(i-hand_ind) == 1 % Maintained
            wrench_tmp = fc_opt(ind:indf,:);
        elseif c_types(i-hand_ind) == 3 % Sliding
            n_i = Cn_tot(i,:).';
            p_i = Cp_tot(i,:);
            G_s_i = build_g(p_i, 1);
            H_s_i = build_h(0,0,1,p_i);
            c_e_p_i = H_s_i*G_s_i'*d_pose;
            norm_force = fc_opt(ind:indf,:)*n_i;
            tang_force = - mu_env * norm(norm_force)*(c_e_p_i / norm(c_e_p_i));
            wrench_tmp = norm_force + tang_force;
        else
            error('Dont know what to do here! Not supposed to get this!');
        end
    end
    
    % Adding the moment part if soft finger
    if cf_dim_tot(i) == 4 % if i soft finger contact
        indmf = indm + 1;
        if i <= num_hand_conts % if still hand contacts
            wrench_tmp = [wrench_tmp; fc_opt(indm)]; % Add the moment part
        end
    end
    
    % Updating the total wrench vector
    wrench_opt_tot = [wrench_opt_tot; wrench_tmp];
    
    % Update the starting index for next iteration
    ind = indf+1;
    
    % if i still hand soft finger contact
    if cf_dim_tot(i) == 4
        indm = indmf + 1; % update the moment index
    end
    % When hand contacts are finished, with the force index jump the moment
    % part
    if soft_fing && i == length(find(cf_dim_tot == 4))
        ind = ind + num_hand_conts;
    end
    
end 

% Trasforming the opt_force in matrix with forces on rows for plotting
ind = 1;
Cf = [];
Mf = []; % NOT RETURNED AS OF NOW
for i = 1: size(Cn_tot,1)
    
    indf = ind+3-1;
    indm = indf +1;
    
    Cf = [Cf; wrench_opt_tot(ind:indf,:).'];
    ind = indf+1;
    
    % Soft finger -> then skip the moment part and save the moment
    if cf_dim_tot(i) == 4 
        ind = ind+1;
        Mf = [Mf; wrench_opt_tot(ind:indf,:).'];
    else
        Mf = [Mf; 0]; % if not soft finger, add zero
    end
    
end

% Checking which forces do not comply with the constraints
indexes_viol = find(sigma_leq > Delta);
ind = 1;
Cp_viol = [];
Cf_viol = [];

% NOT WORKING AS OF NOW
% for i = 1:size(Cf,1)
%     indf = ind+3-1;
%     if any(ismember([ind:indf],indexes_viol)) 
%         Cf_viol = [Cf_viol; Cf(i,:)];
%         Cp_viol = [Cp_viol; Cp_tot(i,:)];
%     end
%     ind = indf+1;
%     if cf_dim_tot(i) == 4 % Soft finger -> then skip the moment part
%         ind = ind+1;
%     end
% end

end

