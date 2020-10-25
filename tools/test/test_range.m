function test_range(M1, M2)
% test_range - Description
%
% Syntax: test_range(M1, M2)
%
% Long description

gauss = rref([M1, M2]);
if ~isempty(nonzeros(gauss(size(M1, 2) + 1 : end, :)))
    error(' ')
end
end