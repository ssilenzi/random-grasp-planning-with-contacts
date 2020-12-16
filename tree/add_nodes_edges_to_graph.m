function [G_out] = add_nodes_edges_to_graph(G,nodes_in,edges_in,ID_s)

% ADD NODES EDGES TO GRAPH - Adds the sequence of nodes and edges to the
% graph G. The first node in nodes_in is connected to the node with ID_s of
% the graph G.

% Adding the newly created nodes
G = addnode(G,nodes_in);

% Adding the corresponding weighted edges
[ID_next, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~] = ...
    get_node_properties(nodes_in(1,:));
G = addedge(G,ID_s,ID_next,edges_in(1,:));  % adding first edge

if height(edges_in) >= 2                    % adding the other edges if any
    for j = 2:height(edges_in)
        ID_prev = ID_next;
        ID_next = ID_next + 1; % not getting here but should be ok
        G = addedge(G,ID_prev,ID_next,edges_in(j,:)); % adding other edges
    end
end

G_out = G;

end

