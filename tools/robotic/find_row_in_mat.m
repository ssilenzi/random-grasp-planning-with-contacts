function index = find_row_in_mat(M, r, epsilon)
% FINDROWINMAT This function finds a row in a matrix

if ~exist('epsilon', 'var')
    epsilon = 0.0001;
end

for i=1:size(M,1)
    if abs(norm(M(i,:) - r)) < epsilon
        index = i;
        return;
    end
end

index = [];
end