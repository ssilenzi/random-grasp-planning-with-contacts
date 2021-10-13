function s_tot = plot_box_face(box, i_face, al)
if ~exist('al','var')
  al = 0.5;
end

s_tot = {};
for i = 1:length(i_face)
    face = transform_points(box.face_vertex_coordinates{i_face(i)}, box.T);
    % s = fill3(face(:,3), face(:,1), face(:,2), 'r');
    s = fill3(face(:,1), face(:,2), face(:,3), 'c');
    alpha(s, al);
    s_tot{i} = s;
end
end