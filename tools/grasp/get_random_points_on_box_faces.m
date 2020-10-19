function p = get_random_points_on_box_faces(object_state, i_free_faces, ...
    n_points)
% GETRANDOMPOINTSONBOXFACES This function returns "n_points" number of
% random points in the "i_free_faces" faces from the object_state

p = zeros(n_points,3);
points_in_face_total = zeros(length(i_free_faces)*n_points,3);
index_tmp  = [1 1 2 2 3 3];
dimensions = [object_state.l object_state.w object_state.h];

points_counter = 0;
for i=1:length(i_free_faces)
    
    index_tmp2 = [1 2 3];
    points_in_face_local = object_state.FaceVertexCoordinates{...
        i_free_faces(i)};
    dimensions_local = dimensions;
    dimensions_local(index_tmp(i_free_faces(i))) = [];
    index_tmp2(index_tmp(i_free_faces(i))) = [];
    for j=1:n_points
        p_random = -0.5 + rand(1,2);
        
        p_random_face_dimensions = dimensions_local.*p_random;
        p_i = zeros(1,3);
        p_i(index_tmp2) = p_random_face_dimensions;
        p_i(index_tmp(i_free_faces(i))) = points_in_face_local(1, ...
            index_tmp(i_free_faces(i)));
        points_counter = points_counter + 1;
        points_in_face_total(points_counter,:) = p_i;
    end
end

for i=1:n_points
   
    random_index = 1 + round(rand()*(size(points_in_face_total,1)-1));
    p(i,:) = points_in_face_total(random_index,:);
    points_in_face_total(random_index,:) = [];
    
end
end