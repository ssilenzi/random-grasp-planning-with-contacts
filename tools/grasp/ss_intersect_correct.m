function [C] = ss_intersect_correct(A,B)
% function [C]=ss_intesect(A,B);
% Intersection of Range(A) with Range(B)

% C = [];
[ra, ca] = size(A);
[rb, ~] = size(B);
if ra ~= rb
    disp('ERROR ! In Intersect.m: argument must have the same number of rows');
    return;
end
C = null([A -B]);
if ~isempty(C)
    C = orth( A * C(1:ca, :));
else
    C = [];
end