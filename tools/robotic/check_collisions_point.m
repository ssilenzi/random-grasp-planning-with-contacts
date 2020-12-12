function bool = check_collisions_point(box, p)
%CHECK_COLLISIONS_POINT - Description
%
% Syntax: bool = check_collisions_point(box, p)
%
% Long description

T_inv = hom_inv(box.T);
p_local = transform_points(p, T_inv);
x = p_local(1); y = p_local(2); z = p_local(3);
bool = abs(x) < box.l/2 && abs(y) < box.w/2 && abs(z) < box.h/2;
end