function bool = check_collisions_line_sampling(box, p1, p2, points)
%CHECK_COLLISIONS_LINE_SAMPLING - Description
% 
% Syntax: bool = check_collisions_line_sampling(box, p1, p2, points)
%
% Long description

for t = linspace(0, 1, points)
    p = (1-t)*p1 + t*p2;
    bool = check_collisions_point(box, p);
    if bool == true
        return
    end
end
bool = false;
end