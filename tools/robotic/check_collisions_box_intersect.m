function bool = check_collisions_box_intersect(box, env)
%CHECK_COLLISIONS_BOX_INTERSECT
% 
% Syntax: bool = check_collisions_box_intersect(box, env)
% 
% It checks the collisions of the box with the environment using
% projections of faces and intersection of projections.

debug = false;

% check vertices
for i = 1:8
    p = box.T(1:3, 4) + box.T(1:3, 1:3) * box.vertices(i,:).';
    % for every object in the environment:
    for j = 1:size(env, 2)
        % check collisions of the point with the object of the
        % environment
        bool = check_collisions_point(env{j}, p);
        if bool == true
            return
        end
    end
end

% for every object in the environment:
for i = 1:size(env,2)
    % change the coordinates of the vertices of the box in
    % coordinates of the local reference of env{i}
    Tinv = hom_inv(env{i}.T);
    pglobal = box.T * [box.vertices, ones(size(box.vertices,1),1)].';
    plocal = Tinv * pglobal;
    vertices_local = plocal(1:3,:).';
    % identify the faces of the box
    face_vertices_local = cell(1,6);
    for j=1:6
        face_vertices_local{j} = vertices_local(...
            box.face_vertices_indices{j},:);
    end
    % for every face of the box check intersections
    % with the faces 1 and 3 of env{i}:
    env_1_y = env{i}.face_vertices_coordinates{1}(:,2).';
    env_1_z = env{i}.face_vertices_coordinates{1}(:,3).';
    poly_env_1 = polyshape(env_1_y, env_1_z);
    env_3_x = env{i}.face_vertices_coordinates{3}(:,1).';
    env_3_z = env{i}.face_vertices_coordinates{3}(:,3).';
    poly_env_3 = polyshape(env_3_x, env_3_z);
    for j=1:6
        box_yz = unique(face_vertices_local{j}(:,[2,3]), 'rows', 'stable');
        sz = size(box_yz, 1);
        rk = rank(box_yz - box_yz(end,:));
        if sz < 2
            error('A projection of a face can''t be one point or less!')
        elseif sz == 2 || rk == 1
            inters = intersect(poly_env_1, box_yz);
            inters_bool = ~isempty(inters);
        else % sz > 2 && rk == 2
            box_y = box_yz(:,1).';
            box_z = box_yz(:,2).';
            poly_box_yz = polyshape(box_y, box_z);
            inters = intersect(poly_env_1, poly_box_yz);
            inters_bool = ~isempty(inters.Vertices);
        end
        if debug
            fig_inters = figure;
            hold on
            plot(poly_env_1)
            if sz == 2 || rk == 1
                plot(box_yz(:,1).', box_yz(:,2).')
                plot(inters(:,1).', inters(:,2).')
            else % sz > 2 && rk == 2
                plot(poly_box_yz)
                plot(inters)
            end
            % for debugging put a breakpoint here!
            close(fig_inters)
        end
        % if the face j is outside the first projection of env{i},
        % then it is outside env{i} -> go ahead with the next face
        if ~inters_bool
            continue
        end
        % the face j is inside the first projection of env{i}
        % check the second projection of env{i}
        box_xz = unique(face_vertices_local{j}(:,[1,3]), 'rows', 'stable');
        sz = size(box_xz, 1);
        rk = rank(box_xz - box_xz(end,:));
        if sz < 2
            error('A projection of a face can''t be one point or less!')
        elseif sz == 2 || rk == 1
            inters = intersect(poly_env_3, box_xz);
            inters_bool = ~isempty(inters);
        else % sz > 2 && rk == 2
            box_x = box_xz(:,1).';
            box_z = box_xz(:,2).';
            poly_box_xz = polyshape(box_x, box_z);
            inters = intersect(poly_env_3, poly_box_xz);
            inters_bool = ~isempty(inters.Vertices);
        end
        if debug
            fig_inters = figure;
            hold on
            plot(poly_env_3)
            if sz == 2 || rk == 1
                plot(box_xz(:,1).', box_xz(:,2).')
                plot(inters(:,1).', inters(:,2).')
            else % sz > 2 && rk == 2
                plot(poly_box_xz)
                plot(inters)
            end
            % for debugging put a breakpoint here!
            close(fig_inters)
        end
        % if the face j is inside both projections of env{i},
        % then is inside env{i} -> collision -> exit
        if inters_bool
            bool = true;
            return
        end
        % if the face j is inside the first projection of env{i},
        % but outside the second projection of env{i},
        % then it is outside env{i} -> go ahead with the next face
    end
    % the box is outside the object env{i}, go ahead with the next object
    % of the environment
end

% the box is outside the environment -> no collision -> exit
bool = false;
end