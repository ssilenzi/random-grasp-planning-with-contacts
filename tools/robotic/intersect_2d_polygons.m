function i_points = intersect_2d_polygons(A, B)
% intersect2dpolygons Find the intersection of 2D-polygons A and B
% A and B contain the points representing the vertices of the polygons
% take a look at test_polygon_intersection for examples

% First step is to find the convex hull of points in A anb B. This is
% i_points = [];
% because points in A anb B can be usorted

if (size(A,1)>2)
    ia = convhull(A(:,1),A(:,2));
    A = A(ia(1:end-1),:);
end
if (size(B,1)>2)
    ib = convhull(B(:,1),B(:,2));
    B = B(ib(1:end-1),:);
end

% find points Ax inside polygon Bx
[in_a, ~] = inpolygon(A(:,1),A(:,2), B(:,1),B(:,2));
if sum(in_a) == length(A(:,1))
    i_points = A;
    return;
end

[in_b, ~] = inpolygon(B(:,1),B(:,2), A(:,1),A(:,2));
if sum(in_b) == length(B(:,1))
    i_points = B;
    return;
end

% find the points of edges of polygon A intersecting edges of polygon B
Ax = [A;A(1,:)];
Bx = [B;B(1,:)];
[i_pointsx, i_pointsy] = polyxpoly(Ax(:,1),Ax(:,2), Bx(:,1),Bx(:,2));

% collect points
i_points = [i_pointsx i_pointsy];
i_points = append_2d_points(i_points, A(in_a,:));
i_points = append_2d_points(i_points, B(in_b,:));
end

function P = append_2d_points(P1, P2)
    for i=1:size(P2,1)
        if (isempty(find_row_in_mat(P1,P2(i,:))))
            P1 = [P1;P2(i,:)];
        end
    end
    P = P1;
end