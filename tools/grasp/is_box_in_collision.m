function res = is_box_in_collision(environment, object, epsilon)
% ISBOXINCOLLISION This checks for collisions between each box in
% "environment" and the one in "object". If they are "epsilon" close or
% overlaped the result is true, otherwise false

if ~exist('epsilon', 'var')
    epsilon = 0.01;
end

object_fake = build_box(object.l - epsilon, object.w - epsilon, ...
    object.h - epsilon, object.T);
res = false;

for i=1:length(environment)
    T_1_2 = environment{i}.T\object_fake.T;
    T_2_1 = inv(T_1_2);
    
    % check for vertices of each box inside the oject and viceversa
    points_b2_inside_b1 = get_points_in_box(environment{i}, ...
        transform_points(object_fake.vertices, T_1_2)); % expressed in b1
    points_b1_inside_b2 = get_points_in_box(object_fake, ...
        transform_points(environment{i}.vertices, T_2_1)); % expresed in b2
    
    if (size([points_b2_inside_b1; points_b1_inside_b2], 1) > 0)
        res = true;
        return;
    else
        % check if edges intersect each face
        for k=1:6
        % each face of ith-box in the environment vs vertex of object_fake
            face_envi = environment{i}.FaceVertexCoordinates{k};
            face_normal_envi = environment{i}.FaceNormals(k,:);
            for l=1:length(object_fake.EdgeVertexCoordinates)
            % faces in object_fake
                edge = transform_points(...
                    object_fake.EdgeVertexCoordinates{l}, T_1_2);
                [pi, check] = intersect_plane_and_line(...
                        face_normal_envi, face_envi(1,:), ...
                        edge(1,:), edge(2,:));
                    if (check > 0 && check <3 && ...
                            is_point_in_box_face(environment{i}, k, pi))
                        res = true;
                        return;
                    end
            end
        end 
    end
end
end