%BUILD_BOX Builds a "box" object centered at ref_frame_T (4x4 homogeneous
% transform) of given x-length, y-width and z-height 
function box = build_box(length, width, height, ref_frame_T)

box.l = length;
box.w = width;
box.h = height;
box.T = ref_frame_T;

l2 = box.l/2; w2 = box.w/2; h2 = box.h/2; % assign short names
% Build a matrix whose rows are the box vertices
box.vertices = [ l2,  w2, -h2;
                 l2, -w2, -h2;
                 l2, -w2,  h2;
                 l2,  w2,  h2;
                -l2,  w2, -h2;
                -l2, -w2, -h2;
                -l2, -w2,  h2;
                -l2,  w2,  h2];
% Build a list whose elements are vectors of the indices of vertices of
% each face 
vf = cell(1,6);
vf{1} = [1, 2, 3, 4];
vf{2} = [5, 6, 7, 8];
vf{3} = [1, 4, 8, 5];
vf{4} = [2, 3, 7, 6];
vf{5} = [4, 3, 7, 8];
vf{6} = [1, 2, 6, 5];
box.face_vertices_indices = vf;
% Build a list whose elements are matrices of the coordinates of
% vertices of each face 
f = cell(1,6);
for i=1:6
    f{i} = box.vertices(vf{i},:);
end
box.face_vertices_coordinates = f;
% Build a matrix whose rows are normal unit vectors to each face 
n = [-1,  0,  0;
      1,  0,  0;
      0, -1,  0;
      0,  1,  0;
      0,  0, -1;
      0,  0,  1];
box.face_normal = n;
% Build a list whose elements are vectors of the indices of vertices of
% each edge
ve = cell(1,12);
ve{1}  = [1, 2];
ve{2}  = [1, 4];
ve{3}  = [1, 5];
ve{4}  = [2, 3];
ve{5}  = [2, 6];
ve{6}  = [3, 4];
ve{7}  = [3, 7];
ve{8}  = [4, 8];
ve{9}  = [5, 6];
ve{10} = [5, 8];
ve{11} = [6, 7];
ve{12} = [7, 8];
box.edge_vertices_indices = ve;
% Build a list whose elements are matrices of the coordinates of vertices
% of each edge
e = cell(1,12);
for i=1:12
    e{i} = box.vertices(ve{i},:);
end
box.edge_vertices_coordinates = e;