function [p, corr_faces_fin] = get_random_points_on_box_faces_partial(object_state, ...
    i_free_faces, i_partial, Cp, Cn, n_points)
% GETRANDOMPOINTSONBOXFACESPARTIAL This function returns "n_points" number 
% of random points in the "i_free_faces" faces from the object_state.
% This function is different from get_random_points_on_box_faces in that it
% chooses random points also on free areas of the partially covered faces.
% Also the faces corresponding to the chosen points are returned

p = zeros(n_points,3);
corr_faces = [];
points_in_face_total = zeros(length(i_free_faces)*n_points,3);
index_tmp  = [1 1 2 2 3 3];
dimensions = [object_state.l object_state.w object_state.h];

points_counter = 0;
for i=1:length(i_free_faces)
    
    index_tmp2 = [1 2 3];
    points_in_face_local = object_state.face_vertices_coordinates{...
        i_free_faces(i)};
    dimensions_local = dimensions;
    dimensions_local(index_tmp(i_free_faces(i))) = [];
    index_tmp2(index_tmp(i_free_faces(i))) = [];
    for j=1:n_points
        p_random = -0.5 + rand(1,2);
        
        p_random_face_dimensions = dimensions_local.*p_random;
        p_i = zeros(1,3);
        p_i(index_tmp2) = p_random_face_dimensions;
        p_i(index_tmp(i_free_faces(i))) = points_in_face_local(1, ...
            index_tmp(i_free_faces(i)));
        points_counter = points_counter + 1;
        points_in_face_total(points_counter,:) = p_i;
        corr_faces = [corr_faces, i_free_faces(i)];
    end
    
    % Now that the points have been chosen, check if the face is a
    % partially covered one and discard the points that are not on the free
    % portion
    if any(i_partial == i_free_faces(i))
        % Getting the contacts on the face
        ind_cont = find_all_row_in_mat(Cn,object_state.face_normals(i,:));
        if (isempty(ind_cont))
            continue;
        end
        Rot = object_state.T(1:3,1:3);
        tras = object_state.T(1:3,4);
        conts_face = (inv(Rot)*Cp(ind_cont,:).' ...
            + repmat(-inv(Rot)*tras,1,length(ind_cont))).' ;
        face_i_vert_ind = object_state.face_vertex_indices{i};
        verts_face = object_state.vertices(face_i_vert_ind,:);
        conts_face(:,index_tmp(i)) = [];
        verts_face(:,index_tmp(i)) = [];
        % Sorting to have a ccw order
        [conts_tmp_x, conts_tmp_y] = ...
            sort_points_clockwise(conts_face(:,1), conts_face(:,2));
        [verts_tmp_x, verts_tmp_y] = ...
            sort_points_clockwise(verts_face(:,1), verts_face(:,2));
        
        % Finding the free area by polygon substraction
%         disp('face '); disp(i_free_faces(i));
%         disp('verts_ord '); disp([verts_tmp_x, verts_tmp_y]);
%         disp('conts_ord '); disp([conts_tmp_x, conts_tmp_y]);
        Pfull = polyshape(verts_tmp_x,verts_tmp_y);
%         disp('1 ');
        Pcov = polyshape(conts_tmp_x,conts_tmp_y);
%         disp('2 ');
        Pfree = subtract(Pfull,Pcov);
%         disp('3 ');
        
        % The last obtained points
        p1 = points_in_face_total(points_counter-1,:);
        p2 = points_in_face_total(points_counter,:);
        p1(:,index_tmp(i)) = [];
        p2(:,index_tmp(i)) = [];
        
        % Checking if points in free area; if not, removing
        if (~inpolygon(p1(:,1),p1(:,2),Pfree.Vertices(:,1),Pfree.Vertices(:,2)))
            points_in_face_total(points_counter-1,:) = [];
            corr_faces(points_counter-1) = [];
            points_counter = points_counter-1;
        end
        if (~inpolygon(p2(:,1),p2(:,2),Pfree.Vertices(:,1),Pfree.Vertices(:,2)))
            points_in_face_total(points_counter,:) = [];
            corr_faces(points_counter) = [];
            points_counter = points_counter-1;
        end
    end
    
end

corr_faces_fin = [];

for i=1:n_points
   
%     size(points_in_face_total)
    random_index = 1 + round(rand()*(size(points_in_face_total,1)-1));
    p(i,:) = points_in_face_total(random_index,:);
    corr_faces_fin = [corr_faces_fin, corr_faces(random_index)];
    points_in_face_total(random_index,:) = [];
    corr_faces(random_index) = [];
    
end
end