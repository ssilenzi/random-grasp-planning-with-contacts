function T = troty(a)
   T = [cos(a), 0, -sin(a);0,1,0; sin(a),0, cos(a)];
   T = [T,[0;0;0]];
   T = [T;[0,0,0,1]];
end