function [G]=build_g_correct(Cp, Co, extended)

% function [G]=build_G(Cp):
% Builds the grasp matrix G for a configuration
% with n contact point c_i and normals n_i.
% Contact point coordinates in base frame are passed
% as rows of argument Cp (nx3). Row vectors
% Object coordinates in base frame are passed
% as rows of argument Co (1x3). Row vector
G = [];
if isempty(Cp)
    return
end
if ~exist('extended','var')
  extended=0;
end


[np, ndim] = size(Cp);

% Building minus the positions of the contact wrt object
OP = (Cp - repmat(Co,np,1));

I=eye(ndim);

Gl = [];
for i=1:np
  G = [G I];
end
if ndim == 3
    for i=1:np
      Cx = [0 -OP(i,3) OP(i,2);
        OP(i,3) 0 -OP(i,1);
        -OP(i,2) OP(i,1) 0];
      Gl = [Gl Cx];
    end
end

if ndim == 2
    for i=1:np
        Gl = [Gl -OP(i,2) OP(i,1)];
    end
end

if (extended > 0)
    G = [G zeros(3,3*np);Gl G];
else
    G = [G;Gl];
end


