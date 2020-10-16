function T = trotz(a)
   T = [cos(a), sin(a),0; -sin(a), cos(a), 0;0,0,1];
   T = [T,[0;0;0]];
   T = [T;[0,0,0,1]];
end