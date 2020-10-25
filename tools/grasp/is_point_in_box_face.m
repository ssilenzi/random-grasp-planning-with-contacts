function res = is_point_in_box_face(box, i, point, epsilon)

% ISPOINTINBOXFACE This function returns true if the "point" is inside the
% i-th face of the "box". Otherwise false

res = false;
if ~exist('epsilon', 'var')
    epsilon = 1e-3;
end
    
if (isempty(point))
    return;
end

index_plane = [1 1 2 2 3 3];
plane = box.face_vertex_coordinates{i};
plane_reduced = plane;
plane_reduced(:,index_plane(i)) = [];
p_plane = point; p_plane(index_plane(i)) = [];
maxs = max(plane_reduced) + epsilon;

a = (p_plane(1) <= maxs(1)) && (p_plane(1) >= -maxs(1));
b = (p_plane(2) <= maxs(2)) && (p_plane(2) >= -maxs(2));

c = abs((point(index_plane(i)) - plane(1, index_plane(i)))) < epsilon;

if ( a && b && c)
    res = true;
end
end