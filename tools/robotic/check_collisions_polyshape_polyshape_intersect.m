function bool = check_collisions_polyshape_polyshape_intersect(...
    mypolyshape1, mypolyshape2, debug)
dbstop if warning
%CHECK_COLLISIONS_POLYSHAPE_POLYSHAPE_INTERSECT - Description
%
% Syntax: bool = check_collisions_polyshape_polyshape_intersect(...
%             mypolyshape1, mypolyshape2, debug)
%
% Long description

% debug is an optional argument: default = false
if ~exist('debug', 'var')
    debug = false;
end
if debug
    dbstop in check_collisions_polyshape_polyshape_intersect at 42
end

% check whether mypolyshape2 is a line or a polygon, and construct its
% polyshape and the intersection
proj = mypolyshape2 - mypolyshape2(1,:);
dim_proj = rank(proj);
if dim_proj == 1
    % the projection is a line. choose the first and last point.
    if all(proj(2,:)~=0) % we must divide for a nonzero vector
        to_be_sorted = proj / proj(2,:);
    else
        to_be_sorted = proj / proj(3,:);
    end
    [~, row_indices] = sort(to_be_sorted);
    mypolyshape2 = mypolyshape2([row_indices(1), row_indices(4)], :);
    bool = check_collisions_polyshape_line_intersect(mypolyshape1, ...
        mypolyshape2, debug);
else % dim_proj == 2
    mypolyshape2 = polyshape(mypolyshape2(:,1).', mypolyshape2(:,2).');
    inters = intersect(mypolyshape1, mypolyshape2);
    is_inters = ~isempty(inters.Vertices);
    bool = is_inters;
    
    % debug section: plots the intersection
    if debug
        fig_inters = figure;
        hold on
        plot(mypolyshape1)
        plot(mypolyshape2)
        plot(inters)
        close(fig_inters)
    end
end
end