function t = sample_twist_from_cone(cone, p_edge)
% SAMPLETWISTFROMCONE This function returns a random twist from the cone
% basis

if ~exist('p_edge', 'var')
    p_edge = 0;
end

alpha = rand(size(cone,2),1);

% non-detaching 

% TODO implement the probability of staying in a cone edge

%alpha = alpha - p_edge;
%alpha = alpha + abs(alpha);

t = cone*alpha;
t = t/norm(t);
end