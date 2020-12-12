function bool = check_collisions_line_intersect(box, p1, p2)
%CHECK_COLLISIONS_LINE_INTERSECT - Description
% 
% Syntax: bool = check_collisions_line_intersect(box, p1, p2)
%
% Long description

extremity = false;
extr = 1;

if ~extremity
    extr = 0.8;
end

% check dimensions
if size(p1,1)~=1 || size(p2,1)~=1
   error('p1 and p2 are row vectors') 
end

% change the coordinates of the points in coordinates of the local
% reference of the box
p1p2 = [p1; (1-extr)*p1 + extr*p2];
T_inv = hom_inv(box.T);
p1p2_local = transform_points(p1p2, T_inv);
line_x = p1p2_local(:,[2,3]); % projection yz
line_y = p1p2_local(:,[1,3]); % projection xz
line_z = p1p2_local(:,[1,2]); % projection xy

% faces of the box to which project the object projection x
box_x_y = box.face_vertices_coordinates{1}(:,2).';
box_x_z = box.face_vertices_coordinates{1}(:,3).';
% projection y
box_y_x = box.face_vertices_coordinates{3}(:,1).';
box_y_z = box.face_vertices_coordinates{3}(:,3).';
% projection z
box_z_x = box.face_vertices_coordinates{5}(:,1).';
box_z_y = box.face_vertices_coordinates{5}(:,2).';
% create the polyshape objects of the faces of the box
planebox_x = polyshape(box_x_y, box_x_z);
planebox_y = polyshape(box_y_x, box_y_z);
planebox_z = polyshape(box_z_x, box_z_y);

% check the face x of the box:
bool = check_collisions_polyshape_line_intersect(planebox_x, line_x);
% if the line is outside the projection x of the box, then it is outside
% the box
if ~bool
    return
end
% the line is inside the projection x of the box.
% check the face y of the box:
bool = check_collisions_polyshape_line_intersect(planebox_y, line_y);
% if the line is inside the projection x of the box but outside the
% projection y of the box, then it is outside the box
if ~bool
    return
end
% the line is inside both the projections x and y of the box.
% check the face z of the box:
bool = check_collisions_polyshape_line_intersect(planebox_z, line_z);
% if the line is inside the faces x, y and z of the box, then is inside
% the box -> collision
if bool
    return
end
% the line is inside the projections x and y of the box but outside the
% projection y of the box, then it is outside the box.
end