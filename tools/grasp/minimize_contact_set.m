function [Cp_out,Cn_out] = minimize_contact_set(Cp_in,Cn_in,box)

% MINIMIZE CONTACT SET -  This functions gives the minimum set of contacts
% needed for ease in force-closure computation
% This is to be used to get a reduced set of the number of contacts with 
% the environment
% IDEA: check if there are more than three contacts on a single face, 
% keep only three.

% Transforming the contact points positions to object frame
Inv_T = box.T; 
Inv_T(1:3,1:3) = Inv_T(1:3,1:3).'; Inv_T(1:3,4) = -Inv_T(1:3,4);
Cp_loc = transform_points(Cp_in, Inv_T);

% Getting the faces indexes from Cp_in
i_face = zeros(size(Cp_loc));
n = zeros(size(Cp_loc));
for i=1:size(Cp_loc,1)
    i_face(i,:) = get_faces_from_points_indexes(box, Cp_loc(i,:));
end

% disp('i_face '); disp(i_face);

% Taking the first row, checking if among others there are more than 2
% contacts on same face; if yes remove one (iteratively should keep only 3)
Cp_out = Cp_in;
Cn_out = Cn_in;

epsilon = 1e-4;

n_rows = size(Cn_out, 1);
del_indexes = [];
for i = 1:n_rows
    i_face_id = i_face(i,:);
    count = 0;
    for j = i+1:n_rows
%         disp('i j are '); disp([i j]);
        j_face_id = i_face(i,:);
        % If contact on same face/vectex and same normals, delete
        same_faces = norm(j_face_id - i_face_id) < epsilon;
        same_normals = norm(Cn_out(i,:) - Cn_out(j,:)) < epsilon;
        if (same_faces && same_normals)
            count = count + 1;
            if count > 2
                del_indexes = [del_indexes j];
            end
        end
    end
end

% Deleting the non needed rows
Cp_out(del_indexes,:) = [];
Cn_out(del_indexes,:) = [];

end

