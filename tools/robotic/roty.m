function R = roty(beta)
% ROTY  rotate around Y by BETA
%
%    R = ROTY(BETA)
%
% See also: ROTX, ROTZ, ROT, POS.

% $ID$

R = [cos(beta) 0 sin(beta); ...
             0 1         0; ...
    -sin(beta) 0 cos(beta)];

% this just cleans up little floating point errors around 0 
% so that things look nicer in the display
if ~isa(beta, 'sym')
  R = round(R, 15);
end
end