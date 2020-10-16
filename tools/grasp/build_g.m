function [G]=build_g(Cp, extended)

% function [G]=build_G(Cp):
% Builds the grasp matrix G for a configuration
% with n contact point c_i and normals n_i.
% Contact point coordinates in base frame are passed
% as rows of argument Cp (nx3). 
G = [];
if isempty(Cp)
    return
end
if ~exist('extended','var')
  extended=0;
end


[np, ndim] = size(Cp);

I=eye(ndim);

Gl = [];
for i=1:np
  G = [G I];
end
if ndim == 3
    for i=1:np
      Cx = [0 -Cp(i,3) Cp(i,2);
        Cp(i,3) 0 -Cp(i,1);
        -Cp(i,2) Cp(i,1) 0];
      Gl = [Gl Cx];
    end
end

if ndim == 2
    for i=1:np
        Gl = [Gl -Cp(i,2) Cp(i,1)];
    end
end

if (extended > 0)
    G = [G zeros(3,3*np);Gl G];
else
    G = [G;Gl];
end


