function plot_oriented_iso_box(X, Y, T, RGBColor)
%PLOT_ORIENTED_ISO_BOX function plot an isocube with initial point X1 and
% final point X2 expresed in T
% X \in \mathbb{R}^3 expresed in T
% Y \in \mathbb{R}^3 expresed in T

if nargin == 3
    RGBColor = [rand/2, rand/2, rand/2];
end

% defining vertex
vertex(1,:) = [X(1), X(2), X(3)];
vertex(2,:) = [X(1), Y(2), X(3)];
vertex(3,:) = [Y(1), Y(2), X(3)];
vertex(4,:) = [Y(1), X(2), X(3)];
vertex(5,:) = [X(1), X(2), Y(3)];
vertex(6,:) = [X(1), Y(2), Y(3)];
vertex(7,:) = [Y(1), Y(2), Y(3)];
vertex(8,:) = [Y(1), X(2), Y(3)];

vertex_tmp = T * [vertex.'; ones(1,8)];
vertex = vertex_tmp(1:3,:).';

% plotting lines
lines{1}  = [vertex(1,:);vertex(2,:)];
lines{2}  = [vertex(2,:);vertex(3,:)];
lines{3}  = [vertex(3,:);vertex(4,:)];
lines{4}  = [vertex(4,:);vertex(1,:)];
lines{5}  = [vertex(5,:);vertex(6,:)];
lines{6}  = [vertex(6,:);vertex(7,:)];
lines{7}  = [vertex(7,:);vertex(8,:)];
lines{8}  = [vertex(8,:);vertex(5,:)];
lines{9}  = [vertex(1,:);vertex(5,:)];
lines{10} = [vertex(2,:);vertex(6,:)];
lines{11} = [vertex(3,:);vertex(7,:)];
lines{12} = [vertex(4,:);vertex(8,:)];

l(1)  = line(lines{1}(:,3),  lines{1}(:,1),  lines{1}(:,2));
l(2)  = line(lines{2}(:,3),  lines{2}(:,1),  lines{2}(:,2));
l(3)  = line(lines{3}(:,3),  lines{3}(:,1),  lines{3}(:,2));
l(4)  = line(lines{4}(:,3),  lines{4}(:,1),  lines{4}(:,2));
l(5)  = line(lines{5}(:,3),  lines{5}(:,1),  lines{5}(:,2));
l(6)  = line(lines{6}(:,3),  lines{6}(:,1),  lines{6}(:,2));
l(7)  = line(lines{7}(:,3),  lines{7}(:,1),  lines{7}(:,2));
l(8)  = line(lines{8}(:,3),  lines{8}(:,1),  lines{8}(:,2));
l(9)  = line(lines{9}(:,3),  lines{9}(:,1),  lines{9}(:,2));
l(10) = line(lines{10}(:,3), lines{10}(:,1), lines{10}(:,2));
l(11) = line(lines{11}(:,3), lines{11}(:,1), lines{11}(:,2));
l(12) = line(lines{12}(:,3), lines{12}(:,1), lines{12}(:,2));
set(l, 'Color', RGBColor)
set(l,'LineWidth', 2)
end