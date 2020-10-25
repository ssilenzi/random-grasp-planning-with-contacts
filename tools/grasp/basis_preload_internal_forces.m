function P = basis_preload_internal_forces(A, J)
% basis_preload_internal_forces - Description
%
% Syntax: P = basis_preload_internal_forces(A, J)
%
% Long description

C = null(J.');
Q0 = [A -C];
B0 = null(Q0);
B01 = B0(1:size(A,2),:);
P = orth(A*B01,'real','skipnormalization');
end