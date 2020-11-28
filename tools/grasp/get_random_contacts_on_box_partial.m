function [Cp_glob,Cn_glob] = get_random_contacts_on_box(box_obj,num_conts,Cp0,Cn0,do_plot)
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

% Get accessible faces
i_faces = get_free_box_faces_partial(box_obj, Cp0, Cn0);

disp('i_faces '); disp(i_faces);

if do_plot 
    plot_box_face(box_obj, i_faces);
end 

% Get random points on free object faces
p = get_random_points_on_box_faces_partial(box_obj, i_faces, num_conts);
n = zeros(size(p));
for i=1:size(p,1)
    i_face = get_faces_from_points_indexes(box_obj, p(i,:));
    n(i,:) = box_obj.face_normals(i_face,:);
end

% Transform random points to global reference system and plot
Cp_glob = transform_points(p, box_obj.T);
Cn_glob = transform_vectors(n, box_obj.T);

if do_plot 
    plot_contacts(Cp_glob, Cn_glob, [1 0 1]);
end 

end

