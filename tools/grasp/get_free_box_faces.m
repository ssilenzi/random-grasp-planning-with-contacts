function face_indexes = get_free_box_faces(box, Cp, Cn)
% GETFREEBOXFACES This function returns the indexes of the box faces that
% are completelly free of contacts

face_indexes = [1 2 3 4 5 6];
contact_index_to_delete = [1 1 2 2 3 3];

for i = 1:6
    i_face = find_all_row_in_mat(Cn,box.face_normals(i,:));
    if (isempty(i_face))
        continue;
    end
    Cp_plane = Cp;
    Cp_plane(:, contact_index_to_delete(i)) = [];
    
    if(size(Cp_plane(i_face,:),1)>2)    
        Cp_plane = convhull(Cp_plane(i_face,1), Cp_plane(i_face,2));
        [r,~] = size(Cp_plane);
        Cp_plane(r,:) = [];
    else
        Cp_plane = Cp_plane(i_face,:);
    end

    if (length(Cp_plane())>2)
        face_indexes(i) = 0;
    else
        disp('Working on it');
    end
end

i_to_delete = find(face_indexes == 0);
face_indexes(i_to_delete) = [];
end