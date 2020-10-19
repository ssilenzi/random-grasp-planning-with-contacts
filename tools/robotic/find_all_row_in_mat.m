function index = find_all_row_in_mat(M, r, epsilon)
% FINDROWINMAT This function finds a row in a matrix and returns all the
% rows where r is present

if ~exist('epsilon', 'var')
    epsilon = 0.0001;
end
index = [];

for i=1:size(M,1)
    if abs(norm(M(i,:) - r)) < epsilon
        index = [index; i];
    end
end
end