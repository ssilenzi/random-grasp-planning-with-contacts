function intersection_point = intersect_lines(P0, P1, Q0, Q1, epsilon)
% INTERSECTLINES This function computes the minimal distance betweeen two
% lines defined by point L1 = P0-P1 and L2 = Q0-Q1. It returns the
% intersection point if lines are closer than epsilon.

if ~exist('epsilon', 'var')
    epsilon = 0.003; % this is in meters
end

u=P1-P0; v=Q1-Q0; w0=P0-Q0;
a=u*u.'; b=u*v.'; c=v*v.'; d=u*w0.'; e=v*w0.';
sc=(b*e-c*d)/(a*c-b^2);
tc=(a*e-b*d)/(a*c-b^2);

dist=norm(w0+(sc*u-tc*v));


if (dist < epsilon && sc >= 0 && sc <= 1)
    intersection_point = P0+sc*u;
else
    intersection_point = [];
end
end