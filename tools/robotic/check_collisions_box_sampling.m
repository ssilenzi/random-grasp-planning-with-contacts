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

vertices_global_hom = box.T * [box.vertices, ones(8,1)].';
vertices_global = vertices_global_hom(1:3,:).';
% check vertices
for i = 1:8
    p_global = vertices_global(i,:).';
    % for every object in the environment:
    for j = 1:size(env, 2)
        % check collisions of the point with the object of the
        % environment
        bool = check_collisions_point(env{j}, p_global);
        if bool == true
            coll_type = 'vertex';
            return
        end
    end
end

% sample the edges (vertices excluded!) and choose a point:
for xlocal = [-box.l/2 : res : box.l/2, box.l/2]
    for ylocal = [-box.w/2 : res : box.w/2, box.w/2]
        for zlocal = [-box.h/2 : res : box.h/2, box.h/2]
            if nnz([isequal(abs(xlocal), box.l/2), ...
                    isequal(abs(ylocal), box.w/2), ...
                    isequal(abs(zlocal), box.h/2)]) == 2
                p_global_hom = [xlocal; ylocal; zlocal; 1];
                p_global = box.T * p_global_hom;
                p_global = p_global(1:3);
                % for every object in the environment:
                for i = 1:size(env, 2)
                    % check collisions of the point with the object of the
                    % environment
                    bool = check_collisions_point(env{i}, p_global);
                    if bool == true
                        coll_type = 'edge';
                        return
                    end
                end
            else
                continue
            end
        end
    end
end

% sample the faces (edges excluded!) and choose a point:
for xlocal = [-box.l/2 : res : box.l/2, box.l/2]
    for ylocal = [-box.w/2 : res : box.w/2, box.w/2]
        for zlocal = [-box.h/2 : res : box.h/2, box.h/2]
            if nnz([isequal(abs(xlocal), box.l/2), ...
                    isequal(abs(ylocal), box.w/2), ...
                    isequal(abs(zlocal), box.h/2)]) == 1
                p_global_hom = [xlocal; ylocal; zlocal; 1];
                p_global = box.T * p_global_hom;
                p_global = p_global(1:3);
                % for every object in the environment:
                for i = 1:size(env, 2)
                    % check collisions of the point with the object of the
                    % environment
                    bool = check_collisions_point(env{i}, p_global);
                    if bool == true
                        coll_type = 'face';
                        return
                    end
                end
            else
                continue
            end
        end
    end
end

% the enviromnent doesn't start inside the object, so the last case is
% not important! exit function
bool = false;
coll_type = '';
end