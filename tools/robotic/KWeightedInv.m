function MinvK = KWeightedInv(M, K)
if ~(isreal(K) && isequal(K, K.'))
    error('K matrix must be real and symmetric')
end
[U, S] = eig(K);
D = U * sqrt(S);
MinvK = D * pinv(M*D);
end