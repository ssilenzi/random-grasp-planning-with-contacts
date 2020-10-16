function N=build_n(Cn, dim);

Cn=normal(Cn')';
[rcn,ccn]=size(Cn);
N = [];

for i=1:rcn
    N(i, 1+ccn*(i-1): ccn*i) = Cn(i,:);
end


N=N';