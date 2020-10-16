function T = trotx(a)
   T = [1,0,0;0,cos(a), sin(a); 0,-sin(a), cos(a)];
   T = [T,[0;0;0]];
   T = [T;[0,0,0,1]];
end