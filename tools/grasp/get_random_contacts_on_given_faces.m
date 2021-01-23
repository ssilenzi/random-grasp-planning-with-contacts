function [Cp_glob,Cn_glob] = ...
    get_random_contacts_on_given_faces(box_obj,num_conts,Cp0,Cn0, ...
    given_faces,partial_faces,sym_conts)

% GET RANDOM CONTACTS ON BOX ON GIVEN FACES

% Get random points on given faces
[p, corr_faces] = get_random_points_on_box_faces_partial(box_obj, given_faces, ...
    partial_faces, Cp0, Cn0, num_conts);
n = zeros(size(p));
for i=1:size(p,1)
    n(i,:) = box_obj.face_normals(corr_faces(i),:);
end

% If only the environment is contacting the object, (or with a random 
% probability) then the contacts shall be symmetric.
% THIS IS DONE ONLY IF THE NUM CONTS == 2. If 0 or 1 contacts, start praying

if sym_conts
    p(2,:) = p(1,:); % second pos = first pos
    n(2,:) = n(1,:); % second normal = first normal
    ind = find(n(1,:) ~= 0); % Getting the index of the direction of cont

    % Flipping the position and normal vectors in the found direction
    p(2,ind) = -p(2,ind);
    n(2,ind) = -n(2,ind);
end

% Transform random points to global reference system and plot
Cp_glob = transform_points(p, box_obj.T);
Cn_glob = transform_vectors(n, box_obj.T);

end

