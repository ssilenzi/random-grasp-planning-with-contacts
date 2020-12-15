function [G,ind_sol,nearest] = build_all_direct_twists(G_in, ...
    environment,target,edge_types,edge_weights,ind_sol_in,nearest_in)

G = G_in;
ind_sol = ind_sol_in;
nearest = nearest_in;

% Getting the number of nodes
n_nodes = height(G.Nodes); % no. of rows of table of Nodes

% Iterating through all nodes and implementing direct twist for the nodes
% in which only the hand contacts the object
for i = 1:n_nodes
    
    %disp('Iteration dir twist no. '); disp(i);
    
    % Getting the ith node
    node_s = G.Nodes(i,:); % row corresponding to i
    
    % Getting the start node properties
    [ID_s, ~, ~, Cp_e_s, Cn_e_s, ~, Cont_h_s, ~, ~, ...
        ~, ~, ~] = get_node_properties(node_s);
    
    exit = 0;
    
    % If only hand contacts, implementing direct twist
    if Cont_h_s == true && (isempty(Cp_e_s) && isempty(Cn_e_s))
        
%         disp('Trying to implement a direct solution!')
        
        [exit, nodes_out, edges_out] = ...
            implement_direct_twist2(node_s, environment, ...
            edge_types, edge_weights, target, n_nodes);
        
    end
    
    % Adding stuff only if exit > 0
    if exit > 0
        
        disp('Adding a direct twist nodes sequence!')
        
        % Check the distance of object in last computed node with the target
        [ID_f, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~] = ...
            get_node_properties(nodes_out(end,:));
        
        % Saving the node inside solutions as it is an exact solution
        ind_sol = [ind_sol, ID_f];
        
        % Saving as the nearest node to the target
        nearest = ID_f;
        
        % Adding the newly created nodes
        G = addnode(G,nodes_out);
        
        % Adding the corresponding weighted edges
        [ID_next, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~] = ...
            get_node_properties(nodes_out(1,:));
        G = addedge(G,ID_s,ID_next,edges_out(1,:)); % adding first edge
        for j = 2:height(edges_out)
            ID_prev = ID_next;
            ID_next = ID_next + 1; % not id getting here but should be ok
            G = addedge(G,ID_prev,ID_next,edges_out(j,:)); % adding other edges
        end
                       
    end % else continue with another node
    
end

end