function Q = normal(F)
%  Q = normal(F)
%  Normalizes the columns of F to one.

Q = [];
[~, nc] = size(F);
for i = 1:nc
    norm = sqrt(F(:,i).' * F(:,i));
    if norm ~= 0
        Q = [Q, F(:,i) / norm];
    else
        Q = [Q, F(:,i)];
    end
end