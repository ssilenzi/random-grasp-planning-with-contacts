function handle_box = plot_box(l,w,h, T, RGBColor, filled, al)
% PLOTBOX plot a box of dimensions x=l (length), y = w (width) and z =
% (height) centered at T and colors RGBColor

if ~exist('T','var')
  T = eye(4);
end
if ~exist('al','var')
  al = 0.5;
end
if ~exist('filled','var')
  filled = false;
end
if ~exist('RGBColor','var')
  RGBColor = [rand/2 rand/2 rand/2];
end

handle_box = {};

X = [l/2;w/2;h/2];
Y = -X;
handle_box{1} = plot_oriented_iso_box(X,Y,T, RGBColor);
        hold on
if filled
    box = build_box(l,w,h, T);
    for i=1:6
        points_in_face = transform_points(box.face_vertex_coordinates{i}, T);
        s = fill3(points_in_face(:,3), points_in_face(:,1), points_in_face(:,2), RGBColor);
        alpha(s, al);
    end
    handle_box{2} = s;
end
end