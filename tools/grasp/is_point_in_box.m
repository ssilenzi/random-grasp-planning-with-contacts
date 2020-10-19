function res =  is_point_in_box(Box, point, epsilon)
% ISPOINTINBOX This function returns true if a point is inside a box 
% (within an epsilon) and false otherwise.
% Point is suposed to be expressend in a reference system centered in the
% box.

if ~exist('epsilon','var')
    epsilon = 3e-3;
end
if (isempty(point))
    res = false;
    return;
end
limits = [Box.l/2 + epsilon; Box.w/2 + epsilon; Box.h/2 + epsilon];
res = true;
for i=1:3
    res = res && ( (point(i) <= limits(i)) && (point(i) >= -limits(i)));
end
end

%% Examples
%
% 1)
% p = [0 0 2];
% Box.l = 1;
% Box.w = 1;
% Box.h = 1;
% Box.T = eye(4);
%
% is_point_in_box(Box, p) reult should be false
%
% 2)
% p = [0 0 1];
% Box.l = 1;
% Box.w = 1;
% Box.h = 1;
% Box.T = eye(4);
%
% is_point_in_box(Box, p) reult should be true