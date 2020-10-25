function H = selection_matrix(soft_contacts_dirs, n, l, m)
% selection_matrix - Description
%
% Syntax: H = selection_matrix(soft_contacts_dirs, n, l, m)
%
% Long description

H = zeros(3*(n+l)+m, 6*n);
if isa(soft_contacts_dirs, 'sym')
    H = sym(H);
end
H(1:3*(n+l), 1:3*(n+l)) = eye(3*(n+l));
for i = 1:m
    H(3*(n+l)+i, 3*(n+l)+3*i-2:3*(n+l)+3*i) = soft_contacts_dirs(:, i).';
end
end