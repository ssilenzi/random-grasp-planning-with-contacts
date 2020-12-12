function [Cp, Cn] = get_contacts_and_normals_box_box(box1, box2)
% GETCONTACTSANDNORMALSBOXBOX This function finds the contact points of
% box1 and box 2. Results are expressed in box2

Cp = [];
Cn = [];
T_1_2 = box1.T\box2.T;
T_2_1 = inv(T_1_2);

% First find face-to-face contacts

% Check if there is a collision
points_b2_inside_b1 = get_points_in_box(box1, ...
    transform_points(box2.vertices, T_1_2)); % Expressed in b1
points_b1_inside_b2 = get_points_in_box(box2, ...
    transform_points(box1.vertices, T_2_1)); % Expresed in b2

possible_contacts_expressed_in_b1 = ...
    append_contacts(points_b2_inside_b1, ...
    transform_points(points_b1_inside_b2, T_1_2));
% possible_contacts_expressed_in_b2 = ...
%     transform_points(possible_contacts_expressed_in_b1, T_2_1);
contact_number = size(possible_contacts_expressed_in_b1,1);

parallel_faces = false;

if contact_number < 1
    
    % It can be that boxes are in contact but vertices are not
    % inside the boxes
    for i=1:6
        % here finds the intersection between the line connecting the
        % centers of the two boxes and aech face of one box
        [pi_b1, check_b2] = ...
            intersect_plane_and_line(box1.face_normal(i,:), ...
            box1.face_vertices_coordinates{i}(1,:),[0 0 0], ...
            transform_points([0 0 0], T_1_2));
        if (check_b2 > 0 && check_b2 <3 && is_point_in_box(box1,pi_b1))            % there is intersection (because the 
            % intersection point bewteen the line and the face of box 2
            % belongs to box 1, too.
            break;
        end
    end
    for j=1:6
        [pi_b2, check_b1] = ...
            intersect_plane_and_line(box2.face_normal(j,:), ...
            box2.face_vertices_coordinates{j}(1,:),[0 0 0], ...
            transform_points([0 0 0], T_2_1));
        if (check_b1 > 0 && check_b1 <3 && is_point_in_box(box2,pi_b2))
            break;
        end
    end
    
    normal_b1 = transform_vectors(-box1.face_normal(i,:), T_2_1);
    normal_b2 = box2.face_normal(j,:);
    if (atan2d(norm(cross(normal_b1,normal_b2)), ...
            dot(normal_b1,normal_b2)) == 0 ...
            && norm(pi_b2-transform_points(pi_b1, T_2_1)) <= 0.003 )
            % Faces are parallel and colides
        points_b2_inside_b1 = pi_b1;
        points_b1_inside_b2 = pi_b2;
        contact_number = 3;
    else
        % TODO check for face-edge, edge-edge, vertex-edge collissions
        vertex_vertex_contacts = zeros(10,3); % prelocate memory
        v1 = transform_points(box1.face_vertices_coordinates{i}, T_2_1);
        v2 = box2.face_vertices_coordinates{j};
        useless_index = [1 2 3 4 1];
        % this index is to void out of range indexes
        nc = 0; % counter for new contacts
        for i=1:4
            for j=1:4
                pc = intersect_lines(v1(useless_index(i),:), ...
                    v1(useless_index(i+1),:), v2(useless_index(j),:), ...
                    v2(useless_index(j+1),:));
                if(~isempty(pc) && is_point_in_box_face(box2,j, pc, 0.01))
                    nc = nc + 1;
                    vertex_vertex_contacts(nc,:) = pc;
                end
            end
        end
        % Cancel non zero contacts
        vertex_vertex_contacts = vertex_vertex_contacts(1:nc,:);
        if (~isempty(vertex_vertex_contacts))
            points_b2_inside_b1 = ...
                transform_points(vertex_vertex_contacts, T_1_2);
        else
            return; % no collission between boxes
        end
        
    end
end
% TO check is a box is inside box. If not do nothing.

if contact_number > 2
% It meas that there is for sure a face-face contact (CG:??)
    possible_contacts_expressed_in_b1 = ...
        append_contacts(points_b2_inside_b1, ...
        transform_points(points_b1_inside_b2, T_1_2));
    i_faces_b1 = get_faces_from_points_indexes(box1, ...
        possible_contacts_expressed_in_b1);
        % in box 1 % get indexes of faces in contact
    i_faces_b2 = get_faces_from_points_indexes(box2, ...
        transform_points(possible_contacts_expressed_in_b1, T_2_1));
        % in box 2
    if (size(i_faces_b1,1)> size(i_faces_b2,1)) % find common faces
        i_faces_b2 = get_faces_from_points_indexes(box2, ...
            transform_points(points_b2_inside_b1, T_2_1));
        A_in_b2 = transform_points(...
            box1.face_vertices_coordinates{i_faces_b1}, T_2_1);
        B_in_b2 = box2.face_vertices_coordinates{i_faces_b2};
    elseif (size(i_faces_b1,1)< size(i_faces_b2,1))
        i_faces_b1 = get_faces_from_points_indexes(box1, ...
            transform_points(points_b1_inside_b2, T_1_2));
        A_in_b2 = box2.face_vertices_coordinates{i_faces_b2};
        B_in_b2 = transform_points(...
            box1.face_vertices_coordinates{i_faces_b1}, T_2_1);
    else
        A_in_b2 = box2.face_vertices_coordinates{i_faces_b2};
        B_in_b2 = transform_points(...
            box1.face_vertices_coordinates{i_faces_b1}, T_2_1);
    end
else
    if (size(points_b2_inside_b1,1) > size(points_b1_inside_b2,1))
        i_faces_b1 = get_faces_from_points_indexes(box1, ...
            points_b2_inside_b1); % in box 1
        i_faces_b2 = get_faces_from_points_indexes(box2, ...
            transform_points(points_b2_inside_b1, T_2_1)); % in box 2
    elseif (size(points_b2_inside_b1,1) < size(points_b1_inside_b2,1))
        i_faces_b1 = get_faces_from_points_indexes(box1, ...
            transform_points(points_b1_inside_b2, T_1_2)); % in box 1
        i_faces_b2 = get_faces_from_points_indexes(box2, ...
            points_b1_inside_b2); % in box 2
    else
        i_faces_b1 = get_faces_from_points_indexes(box1, ...
            points_b2_inside_b1); % in box 1
        i_faces_b2 = get_faces_from_points_indexes(box2, ...
            points_b1_inside_b2); % in box 2
        A_in_b2 = transform_points(...
            box1.face_vertices_coordinates{i_faces_b1}, T_2_1);
        B_in_b2 = box2.face_vertices_coordinates{i_faces_b2};
        parallel_faces = true;
    end
    % look for parallel faces
    for c = 1:size(i_faces_b1,1)
        if(parallel_faces == true)
            break;
        end
        for d=1:size(i_faces_b1,2)
            for a=1:size(i_faces_b2,1)
                for b=1:size(i_faces_b2,2)
                    normal_b1 = transform_vectors(...
                        -box1.face_normal(i_faces_b1(c,d),:), T_2_1);
                    normal_b2 = box2.face_normal(i_faces_b2(a,b),:);
                    angle = atan2d(norm(cross(normal_b1,normal_b2)), ...
                        dot(normal_b1,normal_b2));
                    if (abs(angle) < 1e-3)
                        parallel_faces = true;
                        break;
                    end
                end
                if(parallel_faces == true)
                    i_faces_b1 = i_faces_b1(c,d);
                    A_in_b2 = transform_points(...
                        box1.face_vertices_coordinates{i_faces_b1}, T_2_1);
                    i_faces_b2 = i_faces_b2(a,b);
                    B_in_b2 = box2.face_vertices_coordinates{i_faces_b2};
                    break;
                end
                if(parallel_faces == true)
                    break;
                end
            end
            if(parallel_faces == true)
                break;
            end
        end
        if(parallel_faces == true)
            break;
        end
    end
    
    if(~parallel_faces) % edge contact or point contact
        if (size(points_b2_inside_b1,1) >= size(points_b1_inside_b2,1))
        % there is a vertex of box2 in a face box1
            if contact_number == 1 % check if it is edge-face contact
                connecting_vertices = ...
                    get_connecting_vertices_from_box_vertex(box2, ...
                    transform_points(points_b2_inside_b1, T_2_1));
                for i = 1:size(connecting_vertices,1)
                    [~, check_b2] = intersect_plane_and_line(...
                        box1.face_normal(i_faces_b1,:), ...
                        points_b2_inside_b1, points_b2_inside_b1, ...
                        transform_points(connecting_vertices(i,:), T_1_2));
                    if (check_b2 == 2) % edge-face contact
                        points_b2_inside_b1 = [points_b2_inside_b1; ...
                            transform_points(connecting_vertices(i,:), ...
                            T_1_2)];
                        break;
                    end
                end
            end
            
            A_in_b2 = transform_points(points_b2_inside_b1, T_2_1);
            B_in_b2 = transform_points(...
                box1.face_vertices_coordinates{i_faces_b1}, T_2_1);
        else % There is a vertex of box 1 in a face of box 2
            if contact_number == 1 % check if it is edge-face contact
                connecting_vertices = ...
                    get_connecting_vertices_from_box_vertex(box1, ...
                    transform_points(points_b1_inside_b2, T_1_2));
                for i=1:size(connecting_vertices,1)
                    [~, check_b2] = intersect_plane_and_line(...
                        box2.face_normal(i_faces_b2,:), ...
                        points_b1_inside_b2,points_b1_inside_b2, ...
                        transform_points(connecting_vertices(i,:), T_2_1));
                    if (check_b2 == 2)
                        points_b1_inside_b2 = [points_b1_inside_b2;
                            transform_points(connecting_vertices(i,:), ...
                            T_2_1)];
                        break;
                    end
                end
            end
            A_in_b2 = transform_points(...
                box1.face_vertices_coordinates{i_faces_b1}, T_2_1);
            B_in_b2 = points_b1_inside_b2;
        end
    end
    
end

normal_index = find(abs(box1.face_normal(i_faces_b1,:)));
A = A_in_b2; A(:,normal_index) = []; % Project to 2d
B = B_in_b2; B(:,normal_index) = [];
Cp2d = intersect_2d_polygons(A, B);
a = 1:3; a(normal_index) = []; % m contains the indexes of the plane
Cp3d = ones(size(Cp2d,1),3);
Cp3d(:,a) = Cp2d;
Cp3d(:,normal_index) = Cp3d(:,normal_index) * ...
    A_in_b2(1,normal_index); % recover 3d
Cp = [Cp;Cp3d];
for a=1:size(Cp3d,1)
    nb2_in_b1 = box1.face_normal(i_faces_b1,:);
    Cn = [Cn; transform_vectors(-nb2_in_b1, T_2_1)];
end
end