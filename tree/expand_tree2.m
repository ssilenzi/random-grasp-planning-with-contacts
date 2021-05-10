function [G_out,ind_sol,nearest,iters] = expand_tree2(G,environment,target,...
    n_expand,tol,edge_types,edge_weights,p_release,force_params,dt_max)

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
    
    iters = i;
    
    disp('Iteration expand no. '); disp(i);
    
    exit = false;
    
    n_nodes = height(G.Nodes); % no. of rows of table of Nodes
    
    % Sample a random object configuration in the space
    T_rand = rand_samp_T();
    
    % Select among the nearest nodes a random one
    ID_s = find_nearest_node(G, T_rand);
    node_s = G.Nodes(ID_s,:); % row corresponding to r_nodeID_s
    
    % Getting the start node properties
    [ID_s, ~, ~, Cp_e_s, ~, ~, Cont_h_s, Cp_h_s, Cn_h_s, ...
        ~, ~, prev_pos_s] = get_node_properties(node_s);
    
%     disp('Currently processing node: '); disp(ID_s);
    
    % Implement the correct transition according to the case
    if Cont_h_s == false    % IMPLEMENT POSITIONING
        
        % Checking if there are incongruences (i.e. only one among Cp_h
        % or Cn_h is set)
        no_incongr = (isempty(Cp_h_s) && isempty(Cn_h_s)) || ...
            (~isempty(Cp_h_s) && ~isempty(Cn_h_s));
        
        if no_incongr  	% POSITIONING
            
            % Implementing positioning + moving
            [exit, nodes_out, edges_out] = ...
                implement_positioning_moving2(node_s, environment, ...
                force_params, edge_types, edge_weights, ...
                target, n_nodes, dt_max, T_rand);
            
        else            % This should not happen
            msg = ['In this node, one of Cp_h and Cn_h is empty', ...
                'and the other is not... This should not happen'];
            warning(msg);
        end
        
    else                    % IMPLEMENT MOVING OR RELEASE
        
        % Checking if less than 3 or 4 contacts with environment
        less_3_env_cont = size(Cp_e_s,1) < 3;
        
        % Choosing with assigned prob. to implement moving or release
        rand_num = rand;
                       
        if prev_pos_s || less_3_env_cont || rand_num > p_release % MOVING
            
            % Implementing a simple moving
            [exit, nodes_out, edges_out] = ...
                implement_moving2(node_s, environment, ...
                force_params, edge_types, edge_weights, ...
                target, n_nodes, dt_max, T_rand);            
            
        else                                        % RELEASE
            
            % Implementing a release
            [exit, nodes_out, edges_out] = ...
                implement_release2(node_s, environment, ...
                force_params, edge_types, edge_weights, ...
                target, n_nodes);
            
        end
        
    end
    
    % Trying for a direct solution if newly find node has only hand
    % contacts (only if a new node was found)
    
    exit_direct = 0;
    
    if exit > 0
        
        % Getting properties of last found node
        node_last = nodes_out(end,:);
        [ID_int, ~, ~, Cp_e_int, ~, ~, Cont_h_int, ~, ~, ~, ~, ~] = ...
            get_node_properties(node_last);
        
        % Checking for direct solution possibility conditions
        no_env_cont = size(Cp_e_int,1) == 0;
        
        if no_env_cont && Cont_h_int == true       % Implementing direct twist
            [exit_direct, nodes_dir, edges_dir] = ...
                implement_direct_twist2(node_last, environment, ...
                edge_types, edge_weights, target, ...
                ID_int); % new no of nodes = ID of last computed node
            
            % Add event the partial direct nodes and edgest to the previous
            % (even if did not reach target)
            nodes_out = [nodes_out; nodes_dir];
            edges_out = [edges_out; edges_dir];
                
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
    
    % Adding stuff only if a new node was found previously
    if exit > 0
        
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
        
        % Adding the newly created nodes and edges
        G = add_nodes_edges_to_graph(G,nodes_out,edges_out,ID_s);

        % If exit_direct is 2 -> direct solution was found -> return
        if exit_direct == 2
            G_out = G;
            return;
        end
                       
    end % else continue with another node
    
end

G_out = G;

end

