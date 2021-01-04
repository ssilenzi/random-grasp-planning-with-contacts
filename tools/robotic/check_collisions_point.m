function bool = check_collisions_point(box, p, epsil)
%CHECK_COLLISIONS_POINT - Description
%
% Syntax: bool = check_collisions_point(box, p)
%
% Long description

if ~exist('epsil', 'var')
    epsil = 1e-10; % tolerance for > < comparisons
end

Tvertex = [box.T(1:3, 1:3), box.T(1:3, 4) + ...
            box.T(1:3, 1:3) * box.vertices(1, :).';
            0, 0, 0, 1];
Tinv = hom_inv(Tvertex);
pglobal = [p; 1];
plocal = Tinv * pglobal;
plocal = plocal(1:3);
x = plocal(1); y = plocal(2); z = plocal(3);

epsil + z - box.h

bool = (x > -box.l+epsil) && (y > -box.w+epsil) && (epsil+z < box.h) && ...
            (epsil+x < 0) &&      (epsil+y < 0) &&     (z > 0+epsil);
end