function [points_out, indexes] = get_points_in_box(box, points_in)

% GETPOINTSINBOX This function returns the set of points in points_in 
% inside the box. Points are suposed to be expressend in a reference system
% centered in the box.

points_out = zeros(size(points_in));
indexes_all = zeros(size(points_in, 1), 1);
n_points = 0;
for i=1:size(points_in, 1)
    point = points_in(i, :);
    if(is_point_in_box(box, point))
        n_points = n_points + 1;
        points_out(n_points, :) = point;
        indexes_all(i) = 1;
    end
end
 
points_out = points_out(1:n_points,:);
indexes = (find(indexes_all == 1));

end

%% Example
% clear all
% close all
% clc
% 
% 
% box_book.l = 1;
% box_book.w = 3;
% box_book.h = 2;
% box_book.T = eye(4); 
% box_book.T(1,4) = 5.5;
% box_book.T(2,4) = 6.5;
% box_book.T(3,4) = 1;
% box_book.Cp = [];
% box_book.Cn = [];
% 
% box_shelf.l = 10;
% box_shelf.w = 0.5;
% box_shelf.h = 3;
% box_shelf.T = eye(4); 
% box_shelf.T(1,4) = 5;
% box_shelf.T(2,4) = 4.75;
% box_shelf.T(3,4) = 1.5;
% box_shelf.Cp = [];
% box_shelf.Cn = [];
% 
% T_1_2 = inv(box_shelf.T)*box_book.T;
% 
% l2 = box_book.l/2;
% w2 = box_book.w/2;
% h2 = box_book.h/2;
% 
% p1 = T_1_2*[eye(3) [l2;w2;-h2];0 0 0 1];
% p2 = T_1_2*[eye(3) [l2;-w2;-h2];0 0 0 1];
% p3 = T_1_2*[eye(3) [l2;-w2;h2];0 0 0 1];
% p4 = T_1_2*[eye(3) [l2;w2;h2];0 0 0 1];
% p5 = T_1_2*[eye(3) [-l2;w2;-h2];0 0 0 1];
% p6 = T_1_2*[eye(3) [-l2;-w2;-h2];0 0 0 1];
% p7 = T_1_2*[eye(3) [-l2;-w2;h2];0 0 0 1];
% p8 = T_1_2*[eye(3) [-l2;w2;h2];0 0 0 1];
% 
% points  = [ p1(1:3,4).';
%             p2(1:3,4).';
%             p3(1:3,4).';
%             p4(1:3,4).';
%             p5(1:3,4).';
%             p6(1:3,4).';
%             p7(1:3,4).';
%             p8(1:3,4).'];
% 
% 
% [points_out, ~] = get_points_in_box(box_shelf, points);
% 
% 
% figure;
% box_shelf.T = eye(4);
% box_book.T = T_1_2;
% all_boxes = {box_shelf, box_book};
% plot_boxes(all_boxes);
% xlabel('x');
% ylabel('y');
% zlabel('z');
% hold on
% plot3(points_out(:,1), points_out(:,2), points_out(:,3), '*')