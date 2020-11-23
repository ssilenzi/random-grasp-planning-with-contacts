function invT = tinv(T)
%TINV takes a transformation matrix T and returns its inverse.
% Uses the structure of transformation matrices to avoid taking a matrix
% inverse, for efficiency.

R = T(1:3,1:3);
p = T(1:3,4);
invT = [R.', -R.' * p;
        0, 0, 0, 1];
end