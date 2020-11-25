function bool = check_collisions_line(box, p1, p2, points)
%CHECK_COLLISIONS_LINE - Description
% 
% Syntax: bool = check_collisions_line(box, p1, p2, points)
%
% Long description

for t = linspace(0, 1, points)
    p = (1-t)*p1 + t*p2;
    if check_collisions_point(box, p)
        bool = true;
        return
    end
end
bool = false;
end