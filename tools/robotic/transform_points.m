function points_out = transform_points(points_in,T)
% TRANSFORMPOINTS This function tranform points to the reference system 
% of T
points_out = [];
if isempty(points_in)
    return;
end
points_out = (T(1:3,1:3)*points_in' + ...
    T(1:3,4) * ones(1,size(points_in,1))).';
end