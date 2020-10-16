% BUILDBOX Builds a "Box" object centered at RefFrameT (4x4 homogeneous transform)
% of given x-length, y-width and z-height 
function [Box] = build_box(length,width,height,RefFrameT)

Box.l = length;
Box.w = width;
Box.h = height;
Box.T = RefFrameT;

l2 = Box.l/2; w2 = Box.w/2; h2 = Box.h/2; % assign short names
% Build a matrix whose rows are the box vertices
Box.vertices= [[l2  w2 -h2]; [l2 -w2 -h2];[l2 -w2 h2];[l2 w2 h2];
[-l2 w2 -h2];[-l2 -w2 -h2]; [-l2 -w2 h2]; [-l2 w2 h2]];
% Build a list whose elements are vectors of the indices of vertices of each face 
vf{1} = [1 2 3 4]; vf{2} = [5 6 7 8];
vf{3} = [1 4 8 5]; vf{4} = [2 3 7 6];
vf{5} = [3 7 8 4]; vf{6} = [1 2 6 5];
Box.FaceVertexIndices= vf;
% Build a list whose elements are matrices of the coordinates of vertices of each face 
f = cell(6,1);
for i=1:6
    f{i} = Box.vertices(vf{i},:);
end
Box.FaceVertexCoordinates= f;
% Build a matrix whose rows are normal unit vectors to each face 
n= [[-1 0 0];[1 0 0];[0 -1 0];[0 1 0];[0 0 -1];[0 0 1]];
Box.FaceNormals= n;
% Build a list whose elements are vectors of the indices of vertices of
% each edge
ve{1} = [1 2]; ve{2} = [1 4];
ve{3} = [1 5]; ve{4} = [2 3];
ve{5} = [2 6]; ve{6} = [3 4];
ve{7} = [3 7]; ve{8} = [4 8];
ve{9} = [5 6]; ve{10} = [5 8];
ve{11} = [6 7]; ve{12} = [7 8];
Box.EdgeVertexIndices= ve;
% Build a list whose elements are matrices of the coordinates of vertices
% of each edge
e = cell(12,1);
for i=1:12
    e{i} = Box.vertices(ve{i},:);
end
Box.EdgeVertexCoordinates= e;