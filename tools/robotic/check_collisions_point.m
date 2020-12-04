function bool = check_collisions_point(box, p)
%CHECK_COLLISIONS_POINT - Description
%
% Syntax: bool = check_collisions_point(box, p)
%
% Long description

T_inv = hom_inv(box.T);
p_global = [p; 1];
p_local = T_inv * p_global;
p_local = p_local(1:3);
x = p_local(1); y = p_local(2); z = p_local(3);
bool = abs(x) < box.l/2 && abs(y) < box.w/2 && abs(z) < box.h/2;
end