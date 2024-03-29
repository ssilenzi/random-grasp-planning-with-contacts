function R = rotx(phi)
% ROTX  rotate around X by PHI
%
%    R = ROTX(PHI)
%
% See also: ROTY, ROTZ, ROT, POS.

% $ID$

R = [1        0         0; ...
     0 cos(phi) -sin(phi); ...
     0 sin(phi)  cos(phi)];

% this just cleans up little floating point errors around 0 
% so that things look nicer in the display
if ~isa(phi, 'sym')
  R = round(R, 15);
end
end