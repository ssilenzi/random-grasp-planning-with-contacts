function M_inv = k_weighted_inv(M, K)
% K_weighted_inv - Description
%
% Syntax: M_inv = K_weighted_inv(M, K)
%
% Long description

if ~(isreal(K) && isequal(K, K.'))
    error('K matrix must be real and symmetric')
end
[U, S] = eig(K);
D = U * sqrt(S);
M_inv = D * pinv(M*D);
end