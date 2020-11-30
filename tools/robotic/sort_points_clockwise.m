function [x_out,y_out] = sort_points_clockwise(x,y)
% SORT POINTS CLOCKWISE - Given the x and y coords of a set of points (e.g.
% as used in polyshape), sort them to get the coords of the same points
% ordered in the clockwise direction
% https://stackoverflow.com/questions/13935324/sorting-clockwise-polygon-points-in-matlab

% Find the unweighted mean of the vertices
cx = mean(x);
cy = mean(y);

% Find the angles of the points wrt center
a = atan2(y - cy, x - cx);

% Find the indexes of the correct sorted order
[~, order] = sort(a);

% Reorder the coordinates:
x_out = x(order);
y_out = y(order);

end

