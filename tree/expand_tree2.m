function [G_out,ind_sol,nearest] = expand_tree2(G,environment,target,...
    n_expand,tol,edge_types,edge_weights,p_release,force_params)

% EXPAND TREE 2 - expands the tree by implementing spawning, positioning,
% release or moving. Here positioning is not alone and done with moving
%   Inputs:
%   G           - matlab graph of tree to be expanded
%   environment	- list of boxes of the environment
%   target      - target object box for planning
%   n_expand    - max number of iteration for expansion
%   tol         - tolerance for stopping expansion if target pose near
%   edge_types  - array of strings with the type of transitions
%   edge_weights- array with the costs of each transition
%   p_release   - probability of implementing a release and not a moving
%   force_params - a vector with all the force related params
%   Outputs:
%   G_out       - expanded tree until stopping conditions
%   ind_sol    	- indexes of the node sequence of found solution (if no
%               solution it is the nearest node)
%   nearest     - nearest node to the target

% A node of the graph will contain the following properties:
%   - ID        - the node id
%   - Object    - object stucture, containing also its pose T
%   - Robot     - robot class, with its pose and joints and matrices
%   - Cp_e      - contact positions (rows) of the obj with env
%   - Cn_e      - contact normals (rows) of the obj with env
%   - Cone  	- the free motions cone computed from Cp_e and Cn_e
%   - Cont_h  	- bool stating if the hand is contacting the object
%   - Cp_h      - contact positions (rows) of the obj with hand
%   - Cn_h      - contact normals (rows) of the obj with hand
%   - dir       - direction inside cone, which was chosen for expansion
%   - dist      - distance of hom mats from the target
%   - prev_pos  - a bool stating if the the previous edge was a positioning
%   Cp_h and Cn_h better be empty if Cont_h = false

% An edge of the graph will contain the following properties:
%   - EndNodes  - IDs of the two nodes connected by the edge
%   - Type      - type of transition of the edge
%   - Weight    - edge weight

ind_sol = [];   % no solution yet at the beginning
dist_now = inf;     % current distance from the target

% Inside a big loop expand for a maximum number of iterations
for i = 1:n_expand
    
    disp('Iteration expand no. '); disp(i);
    
    exit = false;
    
    % Get a random starting node from the tree
    n_nodes = height(G.Nodes); % no. of rows of table of Nodes
    ID_s = randsample(1:n_nodes,1); % random Node ID
    node_s = G.Nodes(ID_s,:); % row corresponding to r_nodeID_s
    
    % Getting the start node properties
    [ID_s, ~, ~, ~, ~, ~, Cont_h_s, Cp_h_s, ~, ...
        ~, ~, prev_pos_s] = get_node_properties(node_s);
    
%     disp('Currently processing node: '); disp(ID_s);
    
    % Implement the correct transition according to the case
    if Cont_h_s == false    % IMPLEMENT POSITIONING
        
        % Checking if there are incongruences (i.e. only one among Cp_h
        % or Cn_h is set)
        no_incongr = (isempty(Cp_h_s) && isempty(Cp_h_s)) || ...
            (~isempty(Cp_h_s) && ~isempty(Cp_h_s));
        
        if no_incongr  	% POSITIONING
            
            % Implementing positioning + moving
            [exit, nodes_out, edges_out] = ...
                implement_positioning_moving2(node_s, environment, ...
                force_params, edge_types, edge_weights, ...
                target, n_nodes);
            
        else            % This should not happen
            msg = ['In this node, one of Cp_h and Cn_h is empty', ...
                'and the other is not... This should not happen'];
            warning(msg);
        end
        
    else                    % IMPLEMENT MOVING OR RELEASE
        
        % Choosing with assigned prob. to implement moving or release
        rand_num = rand;
        
        if prev_pos_s || rand_num > p_release       % MOVING
            
            % Implementing a simple moving
            [exit, nodes_out, edges_out] = ...
                implement_moving2(node_s, environment, ...
                force_params, edge_types, edge_weights, ...
                target, n_nodes);
            
        else                                        % RELEASE
            
            % Implementing a release
            [exit, nodes_out, edges_out] = ...
                implement_release2(node_s, environment, ...
                force_params, edge_types, edge_weights, ...
                target, n_nodes);
            
        end
        
    end
    
    %     disp('size box_f '); disp(size(box_f));
    %     disp('size robot_f '); disp(size(robot_f));
    %     disp('size Cp_e_f '); disp(size(Cp_e_f));
    %     disp('size Cn_e_f '); disp(size(Cn_e_f));
    %     disp('size Cone_f '); disp(size(Cone_f));
    %     disp('size Cont_h_f '); disp(size(Cont_h_f));
    %     disp('size Cp_h_f '); disp(size(Cp_h_f));
    %     disp('size Cn_h_f '); disp(size(Cn_h_f));
    
    % Adding stuff only if exit is true
    if exit
        
        % Check the distance of object in last computed node with the target
        [ID_f, ~, ~, ~, ~, ~, ~, ~, ~, ~, dist_f, ~] = ...
            get_node_properties(nodes_out(end,:));
        if dist_f < tol % saving the node inside the tol radius
            ind_sol = [ind_sol, ID_f];
        end
        if dist_f < dist_now % saving the nearest node to the target
            nearest = ID_f;
            dist_now = dist_f;
        end
        
        % Adding the newly created nodes
        G = addnode(G,nodes_out);
        
        % Adding the corresponding weighted edges
        if height(edges_out) == 1       % MOVING OR RELEASE
            G = addedge(G,ID_s,ID_f,edges_out);
        elseif height(edges_out) == 2   % POSITIONING AND MOVING
            [ID_int, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~] = ...
            get_node_properties(nodes_out(1,:));
            G = addedge(G,ID_s,ID_int,edges_out(1,:)); % adding first edge
            G = addedge(G,ID_int,ID_f,edges_out(2,:)); % adding second edge
        else                            % ERROR HERE
            disp('Number of edges to be added is '); disp(height(edges_out));
            error('This should not happen! Unwanted no. of edges!');
        end
        
%         disp('Added node with edge '); disp(e_type);
                       
    end % else continue with another node
    
end

G_out = G;

end

