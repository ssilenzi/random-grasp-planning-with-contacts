function TestRange(M1,M2)
gauss = rref([M1, M2]);
if ~isempty(nonzeros(gauss(size(M1,2)+1:end, :)))
    error(' ')
end
end