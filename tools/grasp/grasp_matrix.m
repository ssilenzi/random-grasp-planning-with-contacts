function G = grasp_matrix(contact_positions, H)
% grasp_matrix - Description
%
% Syntax: G = grasp_matrix(contact_positions, H)
%
% Long description

n = size(contact_positions, 2);
Gtilde = zeros(6, 6*n);
if isa(contact_positions, 'sym')
    Gtilde = sym(Gtilde);
end
for i = 1:n
    Gtilde(1:3, 3*i-2:3*i) = eye(3);
    Gtilde(4:6, 3*i-2:3*i) = cross_p_matrix(contact_positions(:,i));
    Gtilde(4:6, 3*n+3*i-2:3*n+3*i) = eye(3);
end
G = Gtilde * H.';
end