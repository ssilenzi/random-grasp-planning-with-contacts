function near_rand_ID = find_nearest_node(G, T_rand)

% FIND_NEAREST_NODE - Find the nodes in G having the nearest config to
% T_rand. Then, among them, choose at random.

n_nodes = height(G.Nodes); % no. of rows of table of Nodes
near_IDs = []; % IDs of the nearest nodes

near_dist = inf;

for i = 1:n_nodes
    
    % Get ith node and check distance from T_rand
    node_i = G.Nodes(i,:); % row corresponding to r_nodeID_s
    [ID_i,box_i,~,~,~,~,~,~,~,~,~,~] = get_node_properties(node_i);
    T_i = box_i.T;
    dist_i = hom_dist(T_i, T_rand);
    
    % If dist_i < near_dist, clear near_IDs and insert this one. 
    % If dist_i = near_dist add to near_IDs. Else, don't do anything
    if dist_i < near_dist
        near_IDs = [];
        near_IDs = ID_i;
        near_dist = dist_i;
    elseif dist_i == near_dist
        near_IDs = [near_IDs ID_i];
    end
    
end

% Choosing at random between nearest
ind = randsample(1:size(near_IDs,2),1);
near_rand_ID = near_IDs(ind);

end