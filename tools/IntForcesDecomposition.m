% Numerical values based on example 7.1 (Simple gripper)
% [Bicchi 1994]
clear
clc

syms a real
C = [0,   0, 2*a;
    0, 2*a, 3*a;
    0, 2*a,   a].';
N = [0, 1, 0;
    0, -sqrt(3)/2, -1/2;
    0, -sqrt(3)/2,  1/2].';
J = [0; -2*a; zeros(10,1)];
K = 20*sym(eye(12));
K(2,2) = (0.05+0.04*a^2)^-1;
K(10:12,10:12) = 100*eye(3);

H = SelectionMatrix(N, size(C, 2), 0, size(C, 2));
G = GraspMatrix(C, H);
A = null(G);
[E, dQ, dU] = BasisActiveInternal(A, G, J, K);
P = BasisPreloadInternal(A, J);

% Is A equivalent to the one in the article?
% If not, display an error
Abis = [0 0 0 -2 0 0;
2 2 0 0 0 0;
1 -1 0 0 0 0;
0 0 0 1 1 0;
-2 0 0 0 0 0;
-1 0 1 0 0 0;
0 0 0 1 -1 0;
0 -2 0 0 0 0;
0 1 -1 0 0 0;
0 0 0 0 0 sqrt(3);
0 0 0 -4*a 2/sqrt(3)*a 1;
0 0 0 4*a 2/sqrt(3)*a 1];
TestRange(A, Abis);
clear Abis

% Is range([E P]) equivalent to range(A)?
TestRange([E,P], A);

% Calculate dq if y = 0.01
w = [0 1 0 -100 0 0].';
y = 0.01 * 2 / 40;
GinvK = KWeightedInv(G, K);
tmin = -double(subs(GinvK,a,50)) * w;
t = tmin + double(E)*y;
dq = double(subs(dQ*E,a,50)) * y;
du = double(subs(dU*E,a,50)) * y;

function S = CrossPMatrix(omega)
% Takes a 3-vector (angular velocity).
% Returns the skew symmetric matrix.
% Example Input:
%{
  omega = [1; 2; 3];
  S = CrossPMatrix(omega)
%}
% Output:
% S =
%     0    -3     2
%     3     0    -1
%    -2     1     0

S = [0, -omega(3), omega(2);
     omega(3), 0, -omega(1);
    -omega(2), omega(1), 0];
end

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

function [E,dQmatrix,dUmatrix] = BasisActiveInternal(A, G, J, K)
Q = [A -K*J K*G.'];
B = null(Q);
B1 = B(1:size(A,2),:);
B2 = B(size(A,2)+1:size(A,2)+size(J,2),:);
B3 = B(size(A,2)+size(J,2)+1:end,:);
if size(B3,1)~=6
    error(' ')
end
E = orth(A*B1,'real','skipnormalization');
if nargout > 1
    dQmatrix = B2*pinv(A*B1);
    dUmatrix = B3*pinv(A*B1);
end
end

function P = BasisPreloadInternal(A, J)
C = null(J.');
Q0 = [A -C];
B0 = null(Q0);
B01 = B0(1:size(A,2),:);
P = orth(A*B01,'real','skipnormalization');
end

function MinvK = KWeightedInv(M, K)
[U, S] = eig(K);
D = U * sqrt(S);
MinvK = D * pinv(M*D);
end

function TestRange(M1,M2)
gauss = rref([M1, M2]);
if ~isempty(nonzeros(gauss(size(M1,2)+1:end, :)))
    error(' ')
end
end