function bool = check_collisions_polyshape_line_intersect(mypolyshape, ...
    myline, debug)
%CHECK_COLLISIONS_POLYSHAPE_LINE_INTERSECT - Description
%
% Syntax: bool = check_collisions_polyshape_line_intersect(...
%             mypolyshape, myline, debug)
%
% Long description

% debug is an optional argument: default = false
if ~exist('debug', 'var')
    debug = false;
end
if debug
    dbstop in check_collisions_polyshape_line_intersect at 45
end

inters = intersect(mypolyshape, myline);
is_inters = ~isempty(inters);
if is_inters
    % check if the edge is on the boundary of the face
    % of the environment
    centroid = mean(inters);
    [~, tmpbool] = isinterior(mypolyshape, centroid);
    is_centroid_interior = ~tmpbool;
end

bool = false;
if is_inters
    if is_centroid_interior
        bool = true;
    end
end

% debug section: plots the intersection
if debug
    fig_inters = figure;
    hold on
    plot(mypolyshape)
    plot(myline(:,1).', myline(:,2).')
    plot(inters(:,1).', inters(:,2).')
    if is_inters
        plot(centroid(1), centroid(2), '*')
    end
    close(fig_inters)
end
end