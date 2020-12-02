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
%   - 

ind_sol = []; % no solution yet at the beginning

% Inside a big loop expand for a maximum number of iterations
for i = 1:n_expand
    
    % Get a random starting node from the tree
    n_nodes = height(G.Nodes); % no. of rows of table of Nodes
    r_nodeID_s = randsample([1:n_nodes],1); % random Node ID
    node_s = G.Nodes(r_nodeID_s,:); % row corresponding to r_nodeID_s
    
    % Get the properties of the start node (not the global start)
    box_s = node_s.Object{1};
    robot_s = node_s.Robot{1};
    Cp_e_s = node_s.Cp_e{1};
    Cn_e_s = node_s.Cn_e{1};
    Cone_s = node_s.Cone{1};
    Cont_h_s = node_s.Cont_h; % not a cell as only true or false
    Cp_h_s = node_s.Cp_h{1};
    Cn_h_s = node_s.Cn_h{1};
    
    % Creating empty final node (not the global final) properties
 	box_f = [];
    robot_f = [];
    Cp_e_f = [];
    Cn_e_f = [];
    Cone_f = [];
    Cont_h_f = [];
    Cp_h_f = [];
    Cn_h_f = [];
    
    % Implement the correct transition according to the case
    if Cont_h_s == false    % IMPLEMENT SPAWNING OR POSITIONING
        
        if isempty(Cp_h_s) && isempty(Cp_h_s) % SPAWNING
            [box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
                Cont_h_f, Cp_h_f, Cn_h_f] = implement_spawning(...
                box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
                Cont_h_f, Cp_h_f, Cn_h_f);
        elseif ~isempty(Cp_h_s) && ~isempty(Cp_h_s)
            % TODO
            warn('TODO');
        else
            msg = ['In this node, one of Cp_h and Cn_h is empty', ...
                'and the other is not... This should not happen'];
            warning(msg);
        end
        
    else                    % IMPLEMENT MOVING OR RELEASE
        
    end
    
    % Adding the newly created node
    ID_f = n_nodes +1;
    NewNode = table( ID_f, ...
        {box_f}', {robot_f}', {Cp_e_f}', {Cn_e_f}', {Cone_f}', ...
        Cont_h_f, {Cp_h_f}', {Cn_h_f}', ...
        'VariableNames', ...
        {'ID' 'Object' 'Robot' 'Cp_e' 'Cn_e' 'Cone' 'Cont_h' 'Cp_h' 'Cn_h'});
    G = addnode(G,NewNode);
    
    % Adding a weighted edge between node_s and node_f
    NewEdges = table({'moving'}', [10]', ...
        'VariableNames',{'Type','Weight'});
    G = addedge(G,1,2,NewEdges)
    
end

G_out = G;

end

