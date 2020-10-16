function [Q] = normal(F)
%  [Q] = normal(F)
%  Normalizes the columns of F to one.

Q = [];
[nr nc] = size(F);
for i=1:nc
	norm=sqrt(F(1:nr,i)'*F(1:nr,i));
	if norm ~= 0
		Q = [Q F(1:nr,i)/norm];
	else
		Q = [Q F(1:nr,i)];
	end
end