function [graph_out] = append_graphs(graph_up,graph_lo)
% APPEND TWO GRAPHS

% Change the node indexes of graph_lo according to no_nodes of graph_up
n_nodes = height(graph_up.Nodes);
for i = 2:height(graph_lo.Nodes)
    graph_lo.Nodes(i,:).ID = graph_lo.Nodes(i,:).ID + n_nodes;
end

graph_out = addnode(graph_up,graph_lo.Nodes);

% Same for edges
for i = 1:height(graph_lo.Edges)
    end_nodes_i = graph_lo.Edges(i,:).EndNodes;
    for j = 1:2
        if end_nodes_i(j) ~= 1
            end_nodes_i(j) = end_nodes_i(j) + n_nodes - 1;
        end
        edge_i = table(graph_lo.Edges(i,:).Type, graph_lo.Edges(i,:).Weight, ...
            'VariableNames',{'Type','Weight'});
        graph_out = addedge(graph_out, end_nodes_i(1), end_nodes_i(2), edge_i);
    end
end

end

