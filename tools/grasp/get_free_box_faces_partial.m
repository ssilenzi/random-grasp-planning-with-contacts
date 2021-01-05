function [face_indexes, partial_indexes] = get_free_box_faces_partial(box, Cp, Cn)
% GETFREEBOXFACES This function returns the indexes of the box faces that
% are completelly free of contacts

% This is different from the other get_free_box_faces in that here also the
% partially covered faces are returned as free faces. Later, the random
% contact points should be found also on these partially free parts

face_indexes = [1 2 3 4 5 6];
partial_indexes = [];
% remember how the faces are numbered (1 and 2 with normals on x, 3 and 4
% with normals on y and 5 and 6 with normals on z)
contact_index_to_delete = [1 1 2 2 3 3]; 
epsil = 0.001; % threshold for areas comparison (THIS IS CHANGED FOR FRANKA)

for i = 1:6
    % Finding the indexes with cont. normals = face normals
    i_face = find_all_row_in_mat(Cn,box.face_normals(i,:));
    
%     disp('iteration '); disp(i);
%     disp('i_face '); disp(i_face);
    
    % If no contacts on i-th face switch to next face i
    if (isempty(i_face))
        continue;
    end
    
    % Remove the column (coordinate) related to the i-th face normal
    Cp_plane = Cp;
    Cp_plane(:, contact_index_to_delete(i)) = [];
    
%     disp('Cp_plane 1 '); disp(Cp_plane);
    
    % Computing the convex hull to see if the contacts on the considered
    % face form an area >= face area. Only if that's the case, remove the
    % i-th face
    conv_hull_i = [];
    area_i = 0;
    if(size(Cp_plane(i_face,:),1)>2) % 3D case
        [conv_hull_i,area_i] = convhull(Cp_plane(i_face,1), Cp_plane(i_face,2));
        [r,~] = size(conv_hull_i);
        conv_hull_i(r,:) = [];
%         disp('conv_hull_i '); disp(conv_hull_i);
%         disp('area_i '); disp(area_i);
    else % 2D case
        conv_hull_i = Cp_plane(i_face,:);
%         disp('conv_hull_i'); disp(conv_hull_i);
    end
    
    % Getting the vertices of the i-th face
    face_i_vert_ind = box.face_vertex_indices{i};
    face_i_verts = box.vertices(face_i_vert_ind,:);
    % Getting the 2D coordinates of the vertices and computing area
    face_i_verts(:, contact_index_to_delete(i)) = [];
    area_f_i = polyarea(face_i_verts(:,1),face_i_verts(:,2));
    
    is_face_covered = (area_f_i - area_i) < epsil;
%     disp('area_f_i '); disp(area_f_i);
%     disp('area_i '); disp(area_i);

    if (length(conv_hull_i)>2 && is_face_covered)
        face_indexes(i) = 0;
%         disp('face_indexes '); disp(face_indexes);
    elseif (length(conv_hull_i)>2 && ~is_face_covered)
        partial_indexes = [partial_indexes i];
    else 
        % TODO: this case should not be a problem...
        % The area is a line so... in theory there could be contacts on
        % that face
        disp('Line Conv Hull in get_free_box_faces_partial!');
    end
end

i_to_delete = find(face_indexes == 0);
% disp('i_to_delete '); disp(i_to_delete);
face_indexes(i_to_delete) = [];
end