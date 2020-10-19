function vectors_out = transform_vectors(vectors_in,T)
% TRANSFORMVECTORS This function transforms vestors to the reference system
% T
vectors_out = [];
if isempty(vectors_in)
    return;
end
vectors_out = (T(1:3,1:3)*vectors_in')';
end