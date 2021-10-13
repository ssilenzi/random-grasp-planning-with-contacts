function [bool, coll_type] = check_collisions_box(box, env, res, vert)
%CHECK_COLLISIONS_BOX
% 
% Syntax: bool = check_collisions_box(box, env, res)
% 
% Sample the content of the box and check collisions of samples with the
% environment (set of boxes).

if ~exist('res', 'var')
    res = 0.2; % resolution of the square grid
end

if ~exist('vert', 'var')
    vert = true; % check vertices?
end

% check vertices
if vert
    for i = 1:8
        p = box.T(1:3, 4) + box.T(1:3, 1:3) * box.vertices(i,:).';
        % for every object in the environment:
        for j = 1:size(env, 2)
            % check collisions of the point with the object of the
            % environment
            bool = check_collisions_point(env{j}, p);
            if bool == true
                coll_type = 'vertex';
                return
            end
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
                plocal = [xlocal; ylocal; zlocal];
                p = box.T(1:3, 4) + box.T(1:3, 1:3) * plocal;
                % for every object in the environment:
                for i = 1:size(env, 2)
                    % check collisions of the point with the object of the
                    % environment
                    bool = check_collisions_point(env{i}, p);
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
                plocal = [xlocal; ylocal; zlocal];
                p = box.T(1:3, 4) + box.T(1:3, 1:3) * plocal;
                % for every object in the environment:
                for i = 1:size(env, 2)
                    % check collisions of the point with the object of the
                    % environment
                    bool = check_collisions_point(env{i}, p);
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