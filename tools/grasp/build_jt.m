function [Jt]=build_jt(Cp,Org,Zax,Rel)

% function [Jt]=build_Jt(Cp,Cn,Org,Zax,Rel,l):
%
% Builds the ``Jacobian'' matrix Jt for a configuration with n contact 
% point c_i. 
% Contact point coordinates in base frame are passed as rows of Cp (nx3);
% Origin and Z-axis vectors of joint frames are passed as 
% rows of Org (mx3) and Zax (mx3), resp. Matrix Rel (nxm) has
% Rel(i,j) = 0 if the i-th contact does not affect the j-th joint torque;
% Rel(i,j) = 1 if the j-th joint is rotoidal;
% Rel(i,j) = 2 if the j-th joint is prismatic.

[n ndim] = size(Cp);
[m ndim] = size(Org);
Zax=(normal(Zax.')).';
Jt = [];
Jt2 = [];
for j=1:m
 Jtrow = [];
 for i=1:n
   if Rel(i,j) == 0  D = [0 0 0];
    elseif Rel(i,j) == 1  
      CmO = Cp(i,:)-Org(j,:);
      CmOx = [0 -CmO(3) CmO(2); CmO(3) 0 -CmO(1); -CmO(2) CmO(1) 0];
      D = Zax(j,:)*CmOx;
    elseif Rel(i,j) == 2  D = Zax(j,:);
   end
   Jtrow = [Jtrow D];
   end
 Jt = [Jt; Jtrow];
end

for j=1:m
 Jtrow = [];
 for i=1:n
   if Rel(i,j) == 0 | Rel(i,j) == 2  d = [0 0 0];
   elseif Rel(i,j) == 1  
      d = Zax(j,:);
   end
   Jtrow = [Jtrow d];
 end
 Jt2 = [Jt2; Jtrow];
end

Jt = [Jt Jt2];