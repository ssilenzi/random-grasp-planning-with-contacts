function [alpha] = create_rand_comb_vector_proj_cone(Cone_s,box_s,p_generators,T_rand)
% CREATE RAND COMB VECTOR - Create the random combination vector for
% sampling inside the Free Motions Cone.

%   INPUTS:
%   - Cone_s            - the matrix with generators of the cone in columns
%   - box_s             - for the present pose
%   - p_generators      - the probability of sampling a generator
%   - T_rand            - rand obj config towards which expand
%   OUTPUT 
%   - alpha             - the combination vector

max_range = 10;
twist_step = 1;      % for direct twist computation

alpha = zeros(size(Cone_s,2),1); % Inizializing as zeros

p_comb = rand; % random number 
if p_comb < p_generators    % selecting a random generator
    ind = randsample(1:size(Cone_s,2),1);
    alpha(ind) = 1;
else	% select rand. vel and proj in cone. Find that alpha
    
%     % Random twist
%     rand_samp = 2*max_range*(rand(6,1) - 0.5);

    % Twist to get to T_rand from T_s
    T_i_f= inv(box_s.T)*T_rand;
    [t, alpha_t] = homtotwist(T_i_f);
    t = ad(box_s.T)*t*alpha_t*twist_step;
    
    rand_samp = t;
    
    % Finding nearest inside cone
    H = 2*(Cone_s.')*Cone_s;
    f = -2*rand_samp.'*Cone_s;
    A = -eye(size(alpha,1));
    b = zeros(size(alpha,1),1);
    options_qp = optimoptions('quadprog', 'Display', 'off');
    alpha = quadprog(H,f,A,b,[],[],[],[],[],options_qp);
    
end

% Normalizing alpha
alpha = alpha/norm(alpha);

end