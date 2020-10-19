% Numerical values based on example 7.1 (Simple gripper)
% [Bicchi 1994]
close all
clear
run(fullfile('..', 'tools', 'resolve_paths.m'))

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