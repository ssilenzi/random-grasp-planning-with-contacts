function bool = check_collisions_point(box, p)
%CHECK_COLLISIONS_POINT - Description
%
% Syntax: bool = check_collisions_point(box, p)
%
% Long description

Tvertex = [box.T(1:3, 1:3), box.T(1:3, 4) + ...
            box.T(1:3, 1:3) * box.vertices(1, :).';
            0, 0, 0, 1];
Tinv = hom_inv(Tvertex);
pglobal = [p; 1];
plocal = Tinv * pglobal;
plocal = plocal(1:3);
x = plocal(1); y = plocal(2); z = plocal(3);
bool = x > -box.l && y > -box.w && z < box.h && ...
            x < 0 &&      y < 0 &&     z > 0;
end