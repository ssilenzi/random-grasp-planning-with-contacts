function [Cp_glob,Cn_glob] = get_random_contacts_on_box_partial(box_obj,num_conts,Cp0,Cn0,do_plot,scale)
% GET RANDOM CONTACTS ON BOX - Given a box, get a num_cont number of
% random contact positions and normals (expressed in global frame) 
%   Inputs:
%   box_obj     - box object with all its properties
%   num_conts   - needed number of contacts
%   Cp0         - present contact positons (rows)
%   Cn0         - present contact normals (rows)
%   do_plot     - if true plot free faces and chosen contacts
%   Outputs:
%   Cp_glob     - Contact positions (rows) in global frame
%   Cn_glob     - Contact normals (rows) in global frame

if ~exist('scale','var')
    scale = 1;
end

% Defining a probability according to which trying symmetric contacts on
% opposite faces 
p_sym_conts = 0.4;

% Get accessible faces
[i_faces, i_partial] = get_free_box_faces_partial(box_obj, Cp0, Cn0);

% disp('i_faces '); disp(i_faces);
% disp('i_partial '); disp(i_partial);

if do_plot 
    plot_box_face(box_obj, i_faces);
end 

% Get random points on free object faces
[p, corr_faces] = get_random_points_on_box_faces_partial(box_obj, i_faces, ...
    i_partial, Cp0, Cn0, num_conts);
n = zeros(size(p));
for i=1:size(p,1)
    n(i,:) = box_obj.face_normals(corr_faces(i),:);
end

% If only the environment is contacting the object, (or with a random 
% probability) then the contacts shall be symmetric.
% THIS IS DONE ONLY IF THE NUM CONTS == 2. If 0 or 1 contacts, start praying

if (rand < p_sym_conts && num_conts == 2) %|| (size(Cp0,1) <= 4 && num_conts == 2)
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

if do_plot 
    plot_contacts(Cp_glob, Cn_glob, [1 0 1], scale);
end 

end

