function points_out = transform_points(points_in, T)
%TRANSFORM_POINTS This function tranform points
% to the reference system of T
points_out = [];
if isempty(points_in)
    return;
end
tmp1 = [points_in, ones(size(points_in,1),1)].';
tmp2 = (T * tmp1).';
points_out = tmp2(:,1:3);
end