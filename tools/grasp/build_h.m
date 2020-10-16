function [H]=build_h(l,m,n,Cn)

% function [H]=build_H(l,m,n,Cn):
% Builds the constraint matrix H for a configuration with
% l complete-constraint contacts
% m soft-finger contacts
% n hard-finger contacts 
% Related normals are passed in Cn (nx3). 

N = l+m+n;
H = zeros(3*N+3*l+m, 6*N);
H = zeros(3*N+3*l+m, 6*N);
for i = 1:(3*N+3*l) H(i,i) = 1; end
if m ~= 0
for i = 1:m 
    H(3*N+3*l+i, 3*N+3*l+1+3*(i-1): 3*N+3*l+3*i) = Cn(l+i,:);
end
end