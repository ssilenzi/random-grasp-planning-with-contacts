function [bool, coll_type] = check_collisions_box(box, environment, ...
    method, resolution)
%CHECK_COLLISIONS_BOX - Description
%
% Syntax: [bool, coll_type] = check_collisions_box(box, environment, ...
%     method, resolution)
%
% Long description

if nargin < 3 % method not specified
    % choose intersect as default collision detection method
    if nargout > 1
        [bool, coll_type] = check_collisions_box_intersect(box, ...
            environment);
    else
        bool = check_collisions_box_intersect(box, environment);
    end
else % method specified
    if ~strcmp(method,'sampling') % method is intersect
        if nargout > 1
            [bool, coll_type] = check_collisions_box_intersect(box, ...
                environment);
        else
            bool = check_collisions_box_intersect(box, environment);
        end
    else % method is sampling
        if ~exist('resolution','var')
            resolution = 0.2; % resolution of the square grid
        end
        if nargout > 1
            [bool, coll_type] = check_collisions_box_sampling(box, ...
                environment, resolution);
        else
            bool = check_collisions_box_sampling(box, environment, ...
                resolution);
        end
    end
end
end

function [bool, coll_type] = check_collisions_box_intersect(box, env)
%CHECK_COLLISIONS_BOX_INTERSECT
% 
% Syntax: bool = check_collisions_box_intersect(env, obj)
% 
% It checks the collisions of the box with the environment using
% projections of faces and intersection of projections.

% set this constant manually if you want to plot collision detection
debug = false;

% looking for a vertex collision
bool = check_collisions_vertices(env, box);
if bool
    if nargout > 1
        coll_type = 'vertex';
    end
    return
end

% looking for an edge collision
bool = check_collisions_edges_intersect(env, box, debug);
if bool
    if nargout > 1
        coll_type = 'edge';
    end
    return
end

% looking for a face collision
bool = check_collisions_faces_intersect(env, box, debug);
if bool
    if nargout > 1
        coll_type = 'face';
    end
    return
end

% no collision detected -> exit
if nargout > 1
    coll_type = '';
end
end

function [bool, coll_type] = check_collisions_box_sampling(box, env, res)
%CHECK_COLLISIONS_BOX_SAMPLING
% 
% Syntax: bool = check_collisions_box_sampling(box, env, res)
% 
% Sample the content of the box and check collisions of samples with the
% environment (set of boxes).

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

function bool = check_collisions_vertices(env, objbox)

vertices_global = transform_points(objbox.vertices, objbox.T);
% for every box in the environment
for i = 1:size(env, 2)
    % for every vertex of the object
    for j = 1:8
        p_global = vertices_global(j,:);
        % check collisions of the vertices of the object with the box
        % of the environment
        bool = check_collisions_point(env{i}, p_global);
        if bool
            % collision of a vertex with the environment -> exit
            return
        end
    end
end
% no collision of vertices detected -> exit
end

function bool = check_collisions_edges_intersect(env, objbox, debug)
%CHECK_COLLISIONS_EDGES_INTERSECT - Description
%
% Syntax: bool = check_collisions_edges_intersect(env, objbox, proj, debug)
%
% Long description

% for every object in the environment
for i = 1:size(env,2)
    % change the coordinates of the vertices of the object in
    % coordinates of the local reference of environment
    T_inv = hom_inv(env{i}.T);
    vertices_local = transform_points(objbox.vertices, T_inv*objbox.T);

    % faces of the environment to which project the object
    % projection x
    env_x_y = env{i}.face_vertices_coordinates{1}(:,2).';
    env_x_z = env{i}.face_vertices_coordinates{1}(:,3).';
    % projection y
    env_y_x = env{i}.face_vertices_coordinates{3}(:,1).';
    env_y_z = env{i}.face_vertices_coordinates{3}(:,3).';
    % projection z
    env_z_x = env{i}.face_vertices_coordinates{5}(:,1).';
    env_z_y = env{i}.face_vertices_coordinates{5}(:,2).';
    planeenv_x = polyshape(env_x_y, env_x_z);
    planeenv_y = polyshape(env_y_x, env_y_z);
    planeenv_z = polyshape(env_z_x, env_z_y);

    % for every edge j of the object check intersections
    % with the faces x, y and z of the environment
    for j=1:12
        % identify the edges of the object box
        edge_vertices_local = vertices_local(...
            objbox.edge_vertices_indices{j},:);
        linebox_x = edge_vertices_local(:,[2,3]); % projection yz
        linebox_y = edge_vertices_local(:,[1,3]); % projection xz
        linebox_z = edge_vertices_local(:,[1,2]); % projection xy
        % check the face x of the environment box:
        bool = check_collisions_polyshape_line_intersect(...
            planeenv_x, linebox_x, debug);
        % if the edge j is outside the projection x of the environment,
        % then it is outside the environment box -> go ahead with the
        % next edge
        if ~bool
            continue
        end
        % the edge j is inside the projection x of the environment.
        % check the face y of the environment box:
        bool = check_collisions_polyshape_line_intersect(...
            planeenv_y, linebox_y, debug);
        % if the edge j is inside the projection x of the environment
        % but outside the projection y of the environment,
        % then it is outside the environment box -> go ahead with the
        % next edge
        if ~bool
            continue
        end
        % the edge j is inside both the projections x and y of the
        % environment.
        % check the face z of the environment box:
        bool = check_collisions_polyshape_line_intersect(...
            planeenv_z, linebox_z, debug);
        % if the edge j is inside the faces x, y and z of the
        % environment, then is inside the environment box -> collision
        % -> exit
        if bool
            return
        end
        % the edge j is inside the projections x and y of the environment
        % but outside the projection y of the environment, then it is
        % outside the environment box -> go ahead with the next edge
    end
end
% the object box is outside the environment -> no collision -> exit
end

function bool = check_collisions_faces_intersect(env, objbox, debug)
%CHECK_COLLISIONS_FACES_INTERSECT - Description
%
% Syntax: bool = check_collisions_faces_intersect(env, objbox, debug)
%
% Long description

% for every object in the environment
for i = 1:size(env,2)
    % change the coordinates of the vertices of the object in
    % coordinates of the local reference of environment
    T_inv = hom_inv(env{i}.T);
    vertices_local = transform_points(objbox.vertices, T_inv*objbox.T);

    % faces of the environment to which project the object
    % projection x
    env_x_y = env{i}.face_vertices_coordinates{1}(:,2).';
    env_x_z = env{i}.face_vertices_coordinates{1}(:,3).';
    % projection y
    env_y_x = env{i}.face_vertices_coordinates{3}(:,1).';
    env_y_z = env{i}.face_vertices_coordinates{3}(:,3).';
    % projection z
    env_z_x = env{i}.face_vertices_coordinates{5}(:,1).';
    env_z_y = env{i}.face_vertices_coordinates{5}(:,2).';
    planeenv_x = polyshape(env_x_y, env_x_z);
    planeenv_y = polyshape(env_y_x, env_y_z);
    planeenv_z = polyshape(env_z_x, env_z_y);
    
    % for every face j of the object check intersections
    % with the faces x, y and z of the environment
    for j=1:6
        % identify the faces of the object box
        face_vertices_local = vertices_local(...
            objbox.face_vertices_indices{j},:);
        box_x = face_vertices_local(:,[2,3]); % projection yz
        box_y = face_vertices_local(:,[1,3]); % projection xz
        box_z = face_vertices_local(:,[1,2]); % projection xy
        % check the face x of the environment box:
        bool = check_collisions_polyshape_polyshape_intersect(...
            planeenv_x, box_x, debug);
        % if the face j is outside the projection x of the environment,
        % then it is outside the environment box -> go ahead with the
        % next face
        if ~bool
            continue
        end
        % the face j is inside the projection x of the environment.
        % check the face y of the environment box:
        bool = check_collisions_polyshape_polyshape_intersect(...
            planeenv_y, box_y, debug);
        % if the face j is inside the projection x of the environment
        % but outside the projection y of the environment,
        % then it is outside the environment box -> go ahead with the
        % next face
        if ~bool
            continue
        end
        % the face j is inside both the projections x and y of the
        % environment.
        % check the face z of the environment box:
        bool = check_collisions_polyshape_polyshape_intersect(...
            planeenv_z, box_z, debug);
        % if the face j is inside the faces x, y and z of the
        % environment, then is inside the environment box -> collision
        % -> exit
        if bool
            return
        end
        % the face j is inside the projections x and y of the environment
        % but outside the projection y of the environment, then it is
        % outside the environment box -> go ahead with the next face
    end
end
% the object box is outside the environment -> no collision -> exit
end