function [G_out,ind_sol] = expand_tree(G,environment,...
    n_expand,tol,edge_types,edge_weights,p_release)
% EXPAND TREE - expands the tree by implementing spawning, positioning,
% release or moving
%   Inputs:
%   G           - matlab graph of tree to be expanded
%   environment	- list of boxes of the environment
%   n_expand    - max number of iteration for expansion
%   edge_types  - array of strings with the type of transitions
%   edge_weights- array with the costs of each transition
%   p_release   - probability of implementing a release and not a moving
%   tol         - tolerance for stopping expansion if target pose near
%   Outputs:
%   G_out       - expanded tree until stopping conditions
%   ind_sol    	- indexes of the node sequence of found solution (if no
%               solution it is empty)


% A node of the graph will contain the following properties:
%   - Object    - object stucture, containing also its pose T
%   - Robot     - robot class, with its pose and joints and matrices
%   - Cp_e      - contact positions (rows) of the obj with env
%   - Cn_e      - contact normals (rows) of the obj with env
%   - Cone  	- the free motions cone computed from Cp_e and Cn_e
%   - Cont_h  	- bool stating if the hand is contacting the object
%   - Cp_h      - contact positions (rows) of the obj with hand
%   - Cn_h      - contact normals (rows) of the obj with hand
%   The last two (Cp_h and Cn_h) better be empty if Cont_h = false

% An edge of the graph will contain the following properties:
%   - EndNodes  - IDs of the two nodes connected by the edge
%   - Type      - type of transition of the edge
%   - Weight    - edge weight

ind_sol = []; % no solution yet at the beginning

% Inside a big loop expand for a maximum number of iterations
for i = 1:n_expand
    
    exit = false;
    
    % Get a random starting node from the tree
    n_nodes = height(G.Nodes); % no. of rows of table of Nodes
    ID_s = randsample([1:n_nodes],1); % random Node ID
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
    
    % Creating empty edge properties
    e_type = 'unknown';
    e_weight = 0;
    
    % Implement the correct transition according to the case
    if Cont_h_s == false    % IMPLEMENT SPAWNING OR POSITIONING
        
        disp('Currently processing node: '); disp(ID_s);
        
        if ID_s == 1     % If initial node IMPLEMENT SPAWNING
            
            % Getting the main properties
            [exit, box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
                Cont_h_f, Cp_h_f, Cn_h_f] = implement_spawning(...
                box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
                Cont_h_s, Cp_h_s, Cn_h_s, environment);
            
            % Getting the edge properties for spawning
            e_type = edge_types{1};
            e_weight = edge_weights(1);
            
        else            % If not initial node IMPLEMENT POSITIONING
            
            % Checking if there are incongruences (i.e. only one among Cp_h
            % or Cn_h is set)
            no_incongr = (isempty(Cp_h_s) && isempty(Cp_h_s)) || ...
                (~isempty(Cp_h_s) && ~isempty(Cp_h_s));
            
            if no_incongr  	% POSITIONING
                
                % Getting the main properties
                [exit, box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
                    Cont_h_f, Cp_h_f, Cn_h_f] = implement_positioning(...
                    box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
                    Cont_h_s, Cp_h_s, Cn_h_s, environment);
                
                % Getting the edge properties for spawning
                e_type = edge_types{2};
                e_weight = edge_weights(2);
                
            else            % This should not happen
                msg = ['In this node, one of Cp_h and Cn_h is empty', ...
                    'and the other is not... This should not happen'];
                warning(msg);
            end
            
        end
        
    else                    % IMPLEMENT MOVING OR RELEASE
        % TODO
        warning('TODO: moving or release');
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
        
        % Adding the newly created node
        ID_f = n_nodes +1;
        NewNode = table( ID_f, ...
            {box_f}', {robot_f}', {Cp_e_f}', {Cn_e_f}', {Cone_f}', ...
            Cont_h_f, {Cp_h_f}', {Cn_h_f}', ...
            'VariableNames', ...
            {'ID' 'Object' 'Robot' 'Cp_e' 'Cn_e' 'Cone' 'Cont_h' 'Cp_h' 'Cn_h'});
        G = addnode(G,NewNode);
        
        % Adding a weighted edge between node_s and node_f
        NewEdge = table({e_type}', [e_weight]', ...
            'VariableNames',{'Type','Weight'});
        G = addedge(G,ID_s,ID_f,NewEdge);
        
    end % else continue with another node
    
end

G_out = G;

end

