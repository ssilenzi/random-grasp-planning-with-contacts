function [G_out,ind_sol,nearest] = expand_tree(G,environment,target,...
    n_expand,tol,edge_types,edge_weights,p_release,force_params)
% EXPAND TREE - expands the tree by implementing spawning, positioning,
% release or moving
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
    
    % Get the properties of the start node (not the global start)
    box_s = node_s.Object{1};
    robot_s = node_s.Robot{1};
    Cp_e_s = node_s.Cp_e{1};
    Cn_e_s = node_s.Cn_e{1};
    Cone_s = node_s.Cone{1};
    Cont_h_s = node_s.Cont_h; % not a cell as only true or false
    Cp_h_s = node_s.Cp_h{1};
    Cn_h_s = node_s.Cn_h{1};
    dir_s = node_s.dir{1};
    prev_pos_s = node_s.prev_pos;
    
    % Setting final node (not the global final) properties
    % (as start at beginning)
    box_f = box_s;
    robot_f = robot_s;
    Cp_e_f = Cp_e_s;
    Cn_e_f = Cn_e_s;
    Cone_f = Cone_s;
    Cont_h_f = Cont_h_s;
    Cp_h_f = Cp_h_s;
    Cn_h_f = Cn_h_s;
    dir_f = dir_s;
    prev_pos_f = prev_pos_s;
    
    % Creating empty edge properties
    e_type = 'unknown';
    e_weight = 0;
    
%     disp('Currently processing node: '); disp(ID_s);
    
    % Implement the correct transition according to the case
    if Cont_h_s == false    % IMPLEMENT POSITIONING
        
        % Checking if there are incongruences (i.e. only one among Cp_h
        % or Cn_h is set)
        no_incongr = (isempty(Cp_h_s) && isempty(Cp_h_s)) || ...
            (~isempty(Cp_h_s) && ~isempty(Cp_h_s));
        
        if no_incongr  	% POSITIONING
            
            % Getting the main properties
            [exit, box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
                Cont_h_f, Cp_h_f, Cn_h_f, dir_f, prev_pos_f] = ...
                implement_positioning(...
                box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
                Cont_h_s, Cp_h_s, Cn_h_s, dir_s, prev_pos_s, environment);
            
            % Getting the edge properties for spawning
            e_type = edge_types{1};
            e_weight = edge_weights(1);
            
        else            % This should not happen
            msg = ['In this node, one of Cp_h and Cn_h is empty', ...
                'and the other is not... This should not happen'];
            warning(msg);
        end
        
    else                    % IMPLEMENT MOVING OR RELEASE
        
        % Choosing with assigned prob. to implement moving or release
        rand_num = rand;
        
        if prev_pos_s || rand_num > p_release     % MOVING
            
            % Getting the main properties
            [exit, box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
                Cont_h_f, Cp_h_f, Cn_h_f, dir_f, prev_pos_f] = ...
                implement_moving(...
                box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
                Cont_h_s, Cp_h_s, Cn_h_s, dir_s, prev_pos_s, ...
                environment, force_params);
            
            % Getting the edge properties for spawning
            e_type = edge_types{2};
            e_weight = edge_weights(2);
            
        else                        % RELEASE
            
            % Getting the main properties
            [exit, box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
                Cont_h_f, Cp_h_f, Cn_h_f, dir_f, prev_pos_f] = ...
                implement_release(...
                box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
                Cont_h_s, Cp_h_s, Cn_h_s, dir_s, prev_pos_s, ...
                environment, force_params);
            
            % Getting the edge properties for spawning
            e_type = edge_types{3};
            e_weight = edge_weights(3);
            
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
        
        ID_f = n_nodes +1;
        
        % Check the distance with the target node
        dist_f = hom_dist(box_f.T, target.T);
        if dist_f < tol % saving the node inside the tol radius
            ind_sol = [ind_sol, ID_f];
        end
        if dist_f < dist_now % saving the nearest node to the target
            nearest = ID_f;
            dist_now = dist_f;
        end
        
        % Adding the newly created node
        NewNode = table( ID_f, ...
            {box_f}', {robot_f}', {Cp_e_f}', {Cn_e_f}', {Cone_f}', ...
            Cont_h_f, {Cp_h_f}', {Cn_h_f}', {dir_f}', dist_f, prev_pos_f, ...
            'VariableNames', ...
            {'ID' 'Object' 'Robot' 'Cp_e' 'Cn_e' 'Cone' 'Cont_h' ...
            'Cp_h' 'Cn_h' 'dir' 'dist' 'prev_pos'});
        G = addnode(G,NewNode);
        
        % Adding a weighted edge between node_s and node_f
        NewEdge = table({e_type}', e_weight, ...
            'VariableNames',{'Type','Weight'});
        G = addedge(G,ID_s,ID_f,NewEdge);
        
%         disp('Added node with edge '); disp(e_type);
                       
    end % else continue with another node
    
end

G_out = G;

end

