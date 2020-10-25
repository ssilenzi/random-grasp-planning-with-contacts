function [E, dQmatrix, dUmatrix] = basis_active_internal_forces(A, G, J, K)
% basis_active_internal_forces - Description
%
% Syntax: [E, dQmatrix, dUmatrix] = basis_active_internal_forces(A, G, J, K)
%
% Long description

Q = [A -K*J K*G.'];
B = null(Q);
B1 = B(1:size(A,2),:);
B2 = B(size(A,2)+1:size(A,2)+size(J,2),:);
B3 = B(size(A,2)+size(J,2)+1:end,:);
if size(B3,1) ~= 6
    error(' ')
end
E = orth(A*B1,'real','skipnormalization');
if nargout > 1
    dQmatrix = B2*pinv(A*B1);
    dUmatrix = B3*pinv(A*B1);
end
end