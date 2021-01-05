function [face_index] = get_faces_from_points_indexes(box, point)

% GETFACESFROMPOINTSINDEXES This function return the faces that contain the
% point in a box, Results are reported in a matrix wich rows are the faces
% containning each point in rows of point
n_points = size(point,1);
face_index = [];
face_found = false;
switch (n_points)
    case 0
        face_index = [];
    case 1
        verices = box.vertices;
        i_point = find_row_in_mat(verices, point);
        if (~isempty(i_point))
        % means that the point is a vertex so is in 3 faces
            face_found =  true;
            switch (i_point)
                case 1
                    face_index = [1 2 6];
                case 2
                    face_index = [1 4 6];
                case 3
                    face_index = [1 4 5];
                case 4
                    face_index = [1 3 5];
                case 5
                    face_index = [2 3 6];
                case 6
                    face_index = [2 4 6];
                case 7
                    face_index = [2 4 5];
                case 8
                    face_index = [2 3 5];
                otherwise
                    face_index = [];
            end
        end
    case 2
        for i=1:2
            verices = box.vertices;
            i_point = find_row_in_mat(verices, point(i,:));
            if (~isempty(i_point))
            % means that the point is a vertex so is in 3 faces
                face_found =  true;
                switch (i_point)
                    case 1
                        face_index = [face_index;[1 2 6]];
                    case 2
                        face_index = [face_index;[1 4 6]];
                    case 3
                        face_index = [face_index;[1 4 5]];
                    case 4
                        face_index = [face_index;[1 3 5]];
                    case 5
                        face_index = [face_index;[2 3 6]];
                    case 6
                        face_index = [face_index;[2 4 6]];
                    case 7
                        face_index = [face_index;[2 4 5]];
                    case 8
                        face_index = [face_index;[2 3 5]];
                    otherwise
                        face_index = [];
                end
            end
        end
end

if face_found
    return;
end

% if we are here means that points are in a face
n_c_points = 0;
for i = 1:6
    % Augment the ith-face of the box1 to make it to become a box
    ab_face = augment_face(box , i, 0.01);
    T_b_2 = inv(ab_face.T)*box.T;
    p = get_points_in_box(ab_face, transform_points(point, T_b_2));
    if size(p,1) > n_c_points
        face_index = i;
        n_c_points = size(p,1);
    end
end

end
