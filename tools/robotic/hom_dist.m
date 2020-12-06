function [distance] = hom_dist(T1,T2)
% HOM DISTANCE - Distance between two homogeneous matrices
%   Sum of distances between rotations and between translations

% Distance between translations
dist_trans = norm(T2(1:3,4) - T1(1:3,4));

% Distance between rotations
% https://math.stackexchange.com/questions/2113634/comparing-two-rotation-matrices
R1 = T1(1:3,1:3);
R2 = T2(1:3,1:3);

R12 = R1.' * R2;

dist_rot = acos( (trace(R12) - 1) / 2 ); % in rad

% Total distance
distance = dist_trans + dist_rot;

end

