function [E, dQmatrix, dUmatrix] = basis_active_internal_forces_2(G, J, K)
% basis_active_internal_forces 2 - Description
%
% Syntax: [E, dQmatrix, dUmatrix] = basis_active_internal_forces(G, J, K)
%
% Long description
% This is different from basis_active_internal_forces in the following:
%   - A is not an input, it is computed inside
%   - neither orth nor abgrab extracts independent columns from a matrix...
%   - here we use licols (a custom function to do that)
% If J = O, it means that the computed basis is not of active (contr.)
% internal forces. But, it is the forces that are caused by object motions.
% In that case, obviously dQ is not to be used.

A = null(G);
Q = [A -K*J K*G.'];
B = null(Q);
B1 = B(1:size(A,2),:);
B2 = B(size(A,2)+1:size(A,2)+size(J,2),:);
B3 = B(size(A,2)+size(J,2)+1:end,:);

if size(B3,1) ~= 6
    error(' The matrix B3 does not have 6 rows! ')
end

E = lincols(A*B1);

if nargout > 1
    dQmatrix = B2*pinv(A*B1);
    dUmatrix = B3*pinv(A*B1);
end
end