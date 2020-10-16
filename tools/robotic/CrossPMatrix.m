function S = CrossPMatrix(omega)
% Takes a 3-vector (angular velocity).
% Returns the skew symmetric matrix.
% Example Input:
%{
  omega = [1; 2; 3];
  S = CrossPMatrix(omega)
%}
% Output:
% S =
%     0    -3     2
%     3     0    -1
%    -2     1     0

S = [0, -omega(3), omega(2);
     omega(3), 0, -omega(1);
    -omega(2), omega(1), 0];
end