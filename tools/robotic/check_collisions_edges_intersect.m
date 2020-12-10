function bool = check_collisions_edges_intersect(envbox, objbox, proj, ...
    debug)
%CHECK_COLLISIONS_EDGES_INTERSECT - Description
%
% Syntax: bool = check_collisions_edges_intersect(envbox, objbox, ...
%    proj, debug)
%
% Long description

% debug is an optional argument: default = false
if ~exist(debug, 'var')
    debug = false;
end

% change the coordinates of the vertices of the object in
% coordinates of the local reference of environment
T_inv = hom_inv(envbox.T);
vertices_global_hom = objbox.T * [objbox.vertices, ones(8,1)].';
vertices_local_hom = T_inv * vertices_global_hom;
vertices_local = vertices_local_hom(1:3,:).';

% identify the edges of the object box
edge_vertices_local = cell(1,12);
for i=1:12
    edge_vertices_local{i} = vertices_local(...
        objbox.edge_vertices_indices{i},:);
end

% for every edge i of the object check intersections
% with the faces a and b of the environment
% choose the faces of the environment box to intersect
switch proj
    case 'xy'
        % projection yz
        env_a_q1 = envbox.face_vertices_coordinates{1}(:,2).';
        env_a_q2 = envbox.face_vertices_coordinates{1}(:,3).';
        % projection xz
        env_b_q1 = envbox.face_vertices_coordinates{3}(:,1).';
        env_b_q2 = envbox.face_vertices_coordinates{3}(:,3).';
    case 'yz'
        % projection xz
        env_a_q1 = envbox.face_vertices_coordinates{1}(:,1).';
        env_a_q2 = envbox.face_vertices_coordinates{1}(:,3).';
        % projection xy
        env_b_q1 = envbox.face_vertices_coordinates{3}(:,1).';
        env_b_q2 = envbox.face_vertices_coordinates{3}(:,2).';
    case 'xz'
        % projection xy
        env_a_q1 = envbox.face_vertices_coordinates{1}(:,1).';
        env_a_q2 = envbox.face_vertices_coordinates{1}(:,2).';
        % projection yz
        env_b_q1 = envbox.face_vertices_coordinates{3}(:,2).';
        env_b_q2 = envbox.face_vertices_coordinates{3}(:,3).';
end
planeenv_a = polyshape(env_a_q1, env_a_q2);
planeenv_b = polyshape(env_b_q1, env_b_q2);

% for every edge i of the object box
for i=1:12
    % choose the faces of the environment to which project the object
    switch proj
        case 'xy'
            edgebox_a = edge_vertices_local{i}(:,[2,3]); % projection yz
            edgebox_b = edge_vertices_local{i}(:,[1,3]); % projection xz
        case 'yz'
            edgebox_a = edge_vertices_local{i}(:,[1,3]); % projection xz
            edgebox_b = edge_vertices_local{i}(:,[1,2]); % projection xy
        case 'xz'
            edgebox_a = edge_vertices_local{i}(:,[1,2]); % projection xy
            edgebox_b = edge_vertices_local{i}(:,[2,3]); % projection yz
    end
    
    % check the face a of the environment box:
    [is_inters, is_centroid_interior] = ...
        check_collisions_polyshape_line_intersect(planeenv_a, ...
        edgebox_a, debug);
    % if the edge i is outside the projection a of the environment, then
    % it is outside the environment box -> go ahead with the next edge
    if ~is_inters
        continue
    else
        % check if the edge i is on the boundary of the face
        % of the environment
        if ~is_centroid_interior
            continue
        end
    end
    % the edge i is in the interior of the face a of the environment.
    % check the face b of the environment box:
    [is_inters, is_centroid_interior] = ...
        check_collisions_polyshape_line_intersect(planeenv_b, ...
        edgebox_b, debug);
    % if the edge i is inside both face a and face b of the environment,
    % then is inside the environment box -> collision -> exit
    if is_inters
        if is_centroid_interior
            bool = true;
            return
        end
    end
    % if the edge i is inside face a of the environment,
    % but outside face b of the environment, then it is outside
    % the environment box -> go ahead with the next edge
end
end