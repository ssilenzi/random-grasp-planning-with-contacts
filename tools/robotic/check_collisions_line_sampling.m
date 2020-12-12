function bool = check_collisions_line_sampling(box, p1, p2, points)
%CHECK_COLLISIONS_LINE_SAMPLING - Description
% 
% Syntax: bool = check_collisions_line_sampling(box, p1, p2, points)
%
% Long description

extremity = false;
extr = 1;

if ~extremity
    extr = 0.8;
end

% check dimensions
if size(p1,1)~=1 || size(p2,1)~=1
   error('p1 and p2 are row vectors') 
end

for t = linspace(0, extr, points)
    p = (1-t)*p1 + t*p2;
    bool = check_collisions_point(box, p);
    if bool == true
        return
    end
end
end