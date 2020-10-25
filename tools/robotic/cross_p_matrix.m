function S = cross_p_matrix(w)
% Takes a 3-vector (angular velocity).
% Returns the skew symmetric matrix.
% Example Input:
%{
  w = [1; 2; 3];
  S = cross_p_matrix(w)
%}
% Output:
% S =
%     0    -3     2
%     3     0    -1
%    -2     1     0

S = [0, -w(3), w(2);
     w(3), 0, -w(1);
    -w(2), w(1), 0];
end