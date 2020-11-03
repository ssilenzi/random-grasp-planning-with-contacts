function R = rotz(alpha)
% ROTZ  rotate around Z by ALPHA
%
%    R = ROTZ(ALPHA)
%
% See also: ROTX, ROTY, ROT, POS.

% $ID$

R = [cos(alpha) -sin(alpha) 0; ...
     sin(alpha)  cos(alpha) 0; ...
              0           0 1];

% this just cleans up little floating point errors around 0 
% so that things look nicer in the display
if ~isa(alpha, 'sym')
  R = round(R, 15);
end
end