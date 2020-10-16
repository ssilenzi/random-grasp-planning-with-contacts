function [A] =  truncate(A,divide,arg)
% truncate(A,divide,arg) --->> A
% Azzera i quasi-zeri di una matrice, cioe' gli elementi arg
% volte piu' piccoli del max. elem. delle colonne (default arg=1e-4).
% Se divide == 1, divide le colonne di A per il minimo elemento.

if nargin < 3 arg=1e-4; 
   if nargin < 2  divide=0; 
   end; 
end
[ra,ca]=size(A); 
for j=1:ca 
 maxAj=max(abs(A(:,j)));
  for i=1:ra 
%     if abs(A(i,j)) < arg*maxAj  
      if abs(A(i,j)) < arg  
       A(i,j) = 0;
    end
  end
end
if divide==1
 for j=1:ca 
  minj = 1e30;
  for i=1:ra 
    if abs(A(i,j)) ~= 0 & abs(A(i,j)) < minj
       minj= abs(A(i,j));
    end
  end
  A(:,j)=A(:,j)/minj;
 end
end