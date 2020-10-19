function [q] = abgram(F)  
%  [Q] = gram(F)
%  Performs Gram-Schmidt orthogonalization of the column
%  vectors of F, and puts them in Q.
%  The new vectors are NOT normalized.
%  Q has as many columns as the rank of F.

%  I wrote this to fix orth(), which is somewhat buggy 

[nr, nc] = size(F);
if nr ~= 0 
  rF=min([nr nc]);    % rank(F) <= rF;
  i=1;
  q=F(1:nr,1);
  % looks for a non-zero column to start with:
  while ((q.'*q) < eps) && (i+1 <= nc)
    q=F(1:nr,i+1);
    i=i+1;
  end
  [~, ncq]=size(q);
  normq2=q.'*q;
  if normq2 < eps
     q = [];
    return
  end
  while (i<=nc) && (ncq < rF)    % stops when enough columns are straight;
    f=F(1:nr,i);
    [nrq, ncq]=size(q);
    for j=1:ncq
      qj=q(1:nrq,j);
      f=f-qj*qj.'*f/normq2(j);     % Gram-Schmidt;
    end
    ftf = f.'*f;
    if ftf > eps        % then the column was independent;
      q = [q, f];    
      normq2 = [normq2; f.'*f];     % saves calculations;
    end
    i=i+1;
  end
else
    q =[];
end
if q == 0
    q = [];
end