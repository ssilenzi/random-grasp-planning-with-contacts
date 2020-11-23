function vectors_out = transform_vectors(vectors_in,T)
%TRANSFORM_VECTORS This function transforms vestors
% to the reference system of T
vectors_out = [];
if isempty(vectors_in)
    return;
end
vectors_out = (T(1:3,1:3)*vectors_in.').';
end