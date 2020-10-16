function H = SelectionMatrix(SoftContactDirs, n, l, m)
H = zeros(3*(n+l)+m, 6*n);
if isa(SoftContactDirs, 'sym')
    H = sym(H);
end
H(1:3*(n+l), 1:3*(n+l)) = eye(3*(n+l));
for i = 1:m
    H(3*(n+l)+i, 3*(n+l)+3*i-2:3*(n+l)+3*i) = SoftContactDirs(:, i).';
end
end