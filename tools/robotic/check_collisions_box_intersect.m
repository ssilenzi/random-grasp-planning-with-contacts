function bool = check_collisions_box_intersect(box, env)
%CHECK_COLLISIONS_BOX_INTERSECT
% 
% Syntax: bool = check_collisions_box_intersect(box, env)
% 
% It checks the collisions of the box with the environment using
% projections of faces and intersection of projections.

debug = false;

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
            return
        end
    end
end

% for every object in the environment:
for i = 1:size(env,2)
    % change the coordinates of the vertices of the box in
    % coordinates of the local reference of env{i}
    T_inv = hom_inv(env{i}.T);
    vertices_local_hom = T_inv * vertices_global_hom;
    vertices_local = vertices_local_hom(1:3,:).';
    % identify the faces of the box
    face_vertices_local = cell(1,6);
    for j=1:6
        face_vertices_local{j} = vertices_local(...
            box.face_vertices_indices{j},:);
    end
    % for every face j of the box check intersections
    % with the faces 1 and 3 of env{i}:
    env_1_y = env{i}.face_vertices_coordinates{1}(:,2).';
    env_1_z = env{i}.face_vertices_coordinates{1}(:,3).';
    poly_env_1 = polyshape(env_1_y, env_1_z);
    env_3_x = env{i}.face_vertices_coordinates{3}(:,1).';
    env_3_z = env{i}.face_vertices_coordinates{3}(:,3).';
    poly_env_3 = polyshape(env_3_x, env_3_z);
    for j=1:6
        box_yz = face_vertices_local{j}(:,[2,3]);
        proj = box_yz - box_yz(4,:);
        dim_proj = rank(proj);
        if dim_proj == 1
            to_be_sorted = proj(1:2,:) / proj(3,:);
            to_be_sorted = [to_be_sorted; 1; 0];
            [~, row_indices] = sort(to_be_sorted);
            box_yz = box_yz([row_indices(1), row_indices(4)], :);
            inters = intersect(poly_env_1, box_yz);
            is_inters = ~isempty(inters);
        else % dim_proj == 2
            box_y = box_yz(:,1).';
            box_z = box_yz(:,2).';
            poly_box_yz = polyshape(box_y, box_z);
            inters = intersect(poly_env_1, poly_box_yz);
            is_inters = ~isempty(inters.Vertices);
        end
        % debug section: plots the intersection
        if debug
            fig_inters = figure;
            hold on
            plot(poly_env_1)
            if dim_proj == 1
                plot(box_yz(:,1).', box_yz(:,2).')
                plot(inters(:,1).', inters(:,2).')
            else % dim_proj == 2
                plot(poly_box_yz)
                plot(inters)
            end
        end
        % if the face j is outside the first projection of env{i},
        % then it is outside env{i} -> go ahead with the next face
        if ~is_inters
            % debug section
            if debug
                % for debugging put a breakpoint here!
                close(fig_inters)
            end
            continue
        else
            % check if the face j is on the boundary of the first
            % projection of env{i}
            if dim_proj == 1
                centroid = mean(inters);
            else % dim_proj == 2
                centroid = mean(inters.Vertices);
            end
            [~, tmpbool] = isinterior(poly_env_1, centroid);
            is_centroid_interior = ~tmpbool;
            % debug section
            if debug
                plot(centroid(1), centroid(2), '*')
                % for debugging put a breakpoint here!
                close(fig_inters)
            end
            if ~is_centroid_interior
                continue
            end
        end
        % the face j is in the interior of the first projection of env{i}
        % check the second projection of env{i}
        box_xz = face_vertices_local{j}(:,[1,3]);
        proj = box_xz - box_xz(4,:);
        dim_proj = rank(proj);
        if dim_proj == 1
            to_be_sorted = proj(1:2,:) / proj(3,:);
            to_be_sorted = [to_be_sorted; 1; 0];
            [~, row_indices] = sort(to_be_sorted);
            box_xz = box_xz([row_indices(1), row_indices(4)], :);
            inters = intersect(poly_env_3, box_xz);
            is_inters = ~isempty(inters);
        else % dim_proj == 2
            box_x = box_xz(:,1).';
            box_z = box_xz(:,2).';
            poly_box_xz = polyshape(box_x, box_z);
            inters = intersect(poly_env_3, poly_box_xz);
            is_inters = ~isempty(inters.Vertices);
        end
        % debug section: plots the intersection
        if debug
            fig_inters = figure;
            hold on
            plot(poly_env_3)
            if dim_proj == 1
                plot(box_xz(:,1).', box_xz(:,2).')
                plot(inters(:,1).', inters(:,2).')
            else % dim_proj == 2
                plot(poly_box_xz)
                plot(inters)
            end
        end
        % if the face j is inside both projections of env{i},
        % then is inside env{i} -> collision -> exit
        if is_inters
            if dim_proj == 1
                centroid = mean(inters);
            else % dim_proj == 2
                centroid = mean(inters.Vertices);
            end
            [~, tmpbool] = isinterior(poly_env_3, centroid);
            is_centroid_interior = ~tmpbool;
            % debug section
            if debug
                plot(centroid(1), centroid(2), '*')
                % for debugging put a breakpoint here!
                close(fig_inters)
            end
            if is_centroid_interior
                bool = true;
                return
            end
        else
            % debug section
            if debug
                % for debugging put a breakpoint here!
                close(fig_inters)
            end
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