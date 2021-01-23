function [G_out,nearest] = expand_wanted_tree_real_robot(G, ...
    environment,target,n_expand,edge_types,edge_weights,dt_max)

% EXPAND WANTED TREE REAL ROBOT

dist_now = inf;     % current distance from the target

% For a no of max expand
for i = 1:n_expand
    
    disp('Iteration expand no. '); disp(i);
    
    exit = false;
    nodes_out = [];
    edges_out = [];
    
    % Get a last node from the tree
    n_nodes = height(G.Nodes);      % no. of rows of table of Nodes
    n_edges = height(G.Edges);      % no. of rows of table of Edges
    node_s = G.Nodes(n_nodes,:);    % row corresponding to n_nodes    
    if n_edges ~= 0
        edge_s = G.Edges(n_edges,:);    % row corresponding to edge_s
        disp(['The last edge is', edge_s.Type{1}]);
    end
    
    % Input to choose the transition to implement
    prompt = ('Which transition to implement? [1] pos, [2] mov, [3] rel, [0] exit. ');
    good = false;
    tr = [];
    while ~good
        tr = input(prompt);
        if (tr ~= 0 && tr ~= 1 && tr ~= 2 && tr ~= 3)
            warning('Insert a good number: [1] pos, [2] mov, [3] rel. ');
        else
            good = true;
        end
    end
    
    if tr == 0
        return;
    end
    
    % Implement according to chosen transition
    if tr == 1
        disp('Chose pos.');
        [exit, nodes_out, edges_out] = ...
            implement_wanted_positioning_real_robot(node_s, environment, ...
            edge_types, edge_weights, target, n_nodes, dt_max);
    elseif tr == 2
        disp('Chose mov.');
        [exit, nodes_out, edges_out] = ...
            implement_wanted_moving_real_robot(node_s, environment, ...
            edge_types, edge_weights, target, n_nodes, dt_max);
    elseif tr == 3
        disp('Chose rel.');
        [exit, nodes_out, edges_out] = ...
            implement_wanted_release_real_robot(node_s, environment, ...
            edge_types, edge_weights, target, n_nodes, dt_max);
    end
    
    % If not good, continue adding
    if exit == 0
        disp('The node was not added, a new iteration now. ');
        continue;
    end
    
    % Is it enough now?
    prompt = ('Enough nodes?? [1] yes, [0] no. ');
    good = false;
    out = [];
    while ~good
        out = input(prompt);
        if (out ~= 0 && out ~= 1)
            warning('Insert a good number for exiting: [1] yes, [0] no. ');
        else
            good = true;
        end
    end
    
    exit_direct = 0;
    if out == 1 % Trying for direct solution when exiting
        
        % Getting properties of last found node
        node_last = nodes_out(end,:);
        [ID_int, ~, ~, Cp_e_int, ~, ~, Cont_h_int, ~, ~, ~, ~, ~] = ...
            get_node_properties(node_last);
        
        % Checking for direct solution possibility conditions
        no_env_cont = size(Cp_e_int,1) == 0;
        
        if no_env_cont && Cont_h_int == true       % Implementing direct twist
            [exit_direct, nodes_dir, edges_dir] = ...
                implement_direct_twist_real_robot(node_last, environment, ...
                edge_types, edge_weights, target, ...
                ID_int); % new no of nodes = ID of last computed node
            
            % Add event the partial direct nodes and edgest to the previous
            % (even if did not reach target)
            nodes_out = [nodes_out; nodes_dir];
            edges_out = [edges_out; edges_dir];
        else
            warning('Could not implement direct solution!'),
        end

    end
    
    % Check the distance of object in last computed node with the target
    [ID_f, ~, ~, ~, ~, ~, ~, ~, ~, ~, dist_f, ~] = ...
        get_node_properties(nodes_out(end,:));
    if dist_f < dist_now % saving the nearest node to the target
        nearest = ID_f;
        dist_now = dist_f;
    end
    
    % Adding the newly created nodes and edges
    G = add_nodes_edges_to_graph(G,nodes_out,edges_out,n_nodes);
    
    % If exit_direct is 2 -> direct solution was found -> return
    if exit_direct == 2
        G_out = G;
        return;
    end
    
end

end

