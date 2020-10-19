function plot_box_face(Box, i_face, al)
if ~exist('al','var')
  al = 0.5;
end

for i = 1:length(i_face)
    face = transform_points(Box.FaceVertexCoordinates{i_face(i)}, Box.T);
    s = fill3(face(:,3), face(:,1), face(:,2), 'r');
    alpha(s, al);
end
end