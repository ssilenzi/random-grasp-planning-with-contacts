function [bool, coll_type] = check_collisions_box_sampling(box, env, res)
%CHECK_COLLISIONS_BOX_SAMPLING
% 
% Syntax: bool = check_collisions_box_sampling(box, env, res)
% 
% Sample the content of the box and check collisions of samples with the
% environment (set of boxes).

if ~exist('res', 'var')
    res = 0.2; % resolution of the square grid
end

vertices_global = transform_points(box.vertices, box.T);

% looking for a vertex collision
for i = 1:8
    p_global = vertices_global(i,:);
    % for every object in the environment:
    for j = 1:size(env, 2)
        % check collisions of the point with the object of the
        % environment
        bool = check_collisions_point(env{j}, p_global);
        if bool == true
            if nargout > 1
                coll_type = 'vertex';
            end
            return
        end
    end
end

% looking for an edge collision
% sample the edges (vertices excluded!) and choose a point:
for xlocal = [-box.l/2 : res : box.l/2, box.l/2]
    for ylocal = [-box.w/2 : res : box.w/2, box.w/2]
        for zlocal = [-box.h/2 : res : box.h/2, box.h/2]
            if nnz([isequal(abs(xlocal), box.l/2), ...
                    isequal(abs(ylocal), box.w/2), ...
                    isequal(abs(zlocal), box.h/2)]) == 2
                p_local = [xlocal, ylocal, zlocal];
                p_global = transform_points(p_local, box.T);
                % for every object in the environment:
                for i = 1:size(env, 2)
                    % check collisions of the point with the object of the
                    % environment
                    bool = check_collisions_point(env{i}, p_global);
                    if bool == true
                        if nargout > 1
                            coll_type = 'edge';
                        end
                        return
                    end
                end
            end
        end
    end
end

% looking for a face collision
% sample the faces (edges excluded!) and choose a point:
for xlocal = [-box.l/2 : res : box.l/2, box.l/2]
    for ylocal = [-box.w/2 : res : box.w/2, box.w/2]
        for zlocal = [-box.h/2 : res : box.h/2, box.h/2]
            if nnz([isequal(abs(xlocal), box.l/2), ...
                    isequal(abs(ylocal), box.w/2), ...
                    isequal(abs(zlocal), box.h/2)]) == 1
                p_local = [xlocal, ylocal, zlocal];
                p_global = transform_points(p_local, box.T);
                % for every object in the environment:
                for i = 1:size(env, 2)
                    % check collisions of the point with the object of the
                    % environment
                    bool = check_collisions_point(env{i}, p_global);
                    if bool == true
                        if nargout > 1
                            coll_type = 'face';
                        end
                        return
                    end
                end
            end
        end
    end
end

% no collision detected -> exit
if nargout > 1
    coll_type = '';
end
end