function G = GraspMatrix(ContactPositions, H_SelectionMatrix)
n = size(ContactPositions, 2);
Gtilde = zeros(6, 6*n);
if isa(ContactPositions, 'sym')
    Gtilde = sym(Gtilde);
end
for i = 1:n
    Gtilde(1:3, 3*i-2:3*i) = eye(3);
    Gtilde(4:6, 3*i-2:3*i) = CrossPMatrix(ContactPositions(:,i));
    Gtilde(4:6, 3*n+3*i-2:3*n+3*i) = eye(3);
end
G = Gtilde * H_SelectionMatrix.';
end