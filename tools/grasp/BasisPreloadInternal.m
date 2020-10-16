function P = BasisPreloadInternal(A, J)
C = null(J.');
Q0 = [A -C];
B0 = null(Q0);
B01 = B0(1:size(A,2),:);
P = orth(A*B01,'real','skipnormalization');
end