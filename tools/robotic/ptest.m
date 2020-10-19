function [answ,a,x] = ptest(A)
% PTEST(A) --->> answ, a
% Test for the intersection of range(A) with the set of vectors
% with positive components. 
% Answ = 2 if there is a "positive" vector in the range space of A.
% Answ = 1 if there is a "semi-positive" vector in the range space of A.
% Answ = 0 if there is not; 

[ra,ca]=size(A);
options = optimoptions('linprog', 'Algorithm', 'interior-point-legacy', 'Preprocess', 'none');
options.Display = 'off';
[x,fval,flag]=linprog(zeros(ca,1),-A,zeros(ra,1),[],[],[],[],[],options);
% x = linprog(f,A,b,Aeq,beq,lb,ub) defines a set of lower and upper bounds 
% on the design variables, x, so that the solution is always in the range 
% lb ? x ? ub. Set Aeq = [] and beq = [] if no equalities exist.
NA=null(A);
x=normal(x); 
a=truncate(A*x);
  if length(find(a>0))==ra answ =2; 
%-%disp('length(find(a>0))==ra');
  elseif ~isempty(NA) answ = 1; x=NA(:,1); a=zeros(ra,1);
%-%       disp('length(NA)');
  elseif length(find(a>=0))==ra && max(abs(x)) ~= 0, answ =1; 
%-%       disp('length(find(a>=0))==ra & max(abs(x)) ~= 0');
  else answ=0; 
  end
end