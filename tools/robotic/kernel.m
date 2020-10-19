function [Q] = kernel(F)  
%  [Q] = kernel(F)
%  Returns an orthogonal basis of the nullspace of F
%  The basis vectors are NOT normalized.
%  Q has as many columns as the nullity of F.

Ft = F.';
Q = o_compl(Ft);
end