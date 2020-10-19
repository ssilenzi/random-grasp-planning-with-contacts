function vertices = get_connecting_vertices_from_box_vertex(box, vertex)
% GETCONNECTINGVERTICESFROMBOXVERTEX This function returns the vertices of
% a box that includes the point vertex

verices_all = box.vertices;
i_point = find_row_in_mat(verices_all, vertex);
if (~isempty(i_point)) % means that the point is a vertex so is in 3 faces
    switch (i_point)
        case 1
            vertex_index = [2 4 5];
        case 2
            vertex_index = [1 3 6];
        case 3
            vertex_index = [2 4 7];
        case 4
            vertex_index = [1 3 8];
        case 5
            vertex_index = [1 6 8];
        case 6
            vertex_index = [2 5 7];
        case 7
            vertex_index = [3 46 8];
        case 8
            vertex_index = [4 5 7];
        otherwise
            vertex_index = [];
    end
end

vertices = verices_all(vertex_index,:);
end