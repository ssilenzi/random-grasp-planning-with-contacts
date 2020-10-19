function [Q] = o_compl(F)  
%  [Q] = o_compl(F)
%  Returns a basis of the orthogonal complement to the range of F
%  The basis vectors are NOT normalized.
%  Q has as many columns as the nullity of F'.

[nrf, ~] = size(F);
F = abgram(F);
[~, nc] = size(F);
F=[F, eye(nrf)];
Q = abgram(F);
[~, ncq] = size(Q);
Q = Q(:, (nc+1):ncq);
end