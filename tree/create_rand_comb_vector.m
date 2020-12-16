function [alpha] = create_rand_comb_vector(Cone_s,p_generators)
% CREATE RAND COMB VECTOR - Create the random combination vector for
% sampling inside the Free Motions Cone.

%   INPUTS:
%   - Cone_s            - the matrix with generators of the cone in columns
%   - p_generators      - the probability of sampling a generator
%   OUTPUT 
%   - alpha             - the combination vector


alpha = zeros(size(Cone_s,2),1); % Inizializing as zeros

p_comb = rand; % random number 
if p_comb < p_generators    % selecting a random generator
    ind = randsample(1:size(Cone_s,2),1);
    alpha(ind) = 1;
else                        % selecting random combination of generators
    sC = size(Cone_s,2);
    for j = 1:sC
        alpha(j) = rand;
    end
end

end

