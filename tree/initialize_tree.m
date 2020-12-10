function [G] = initialize_tree(object_init,object_fin,robot,environment)
% INITIALIZE TREE - Initializes tree with the starting node made of the
% object in the initial position and the hand just loaded (not spawned)
%   Inputs:
%   object_init     - the initial object structure, with its pose
%   object_fin      - the target object structure, with its pose
%   robot           - the robot class, to be spawned
%   environment     - list of boxes of the environment
%   Output          
%   G               - the tree (just initialized with node properties)

% A node of the graph will contain the following properties:
%   - ID        - ID number of the node
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

% Get contacts with the environment
[success, Cp_e, Cn_e] = get_contacts_with_flag(environment, object_init, object_init.T);
if ~success
    error('Could not get the contacts for the initial position!');
end

% Getting the cone and plotting if necessary
% TODO: the dimension d could be passed form outside
Cone = pfc_analysis(Cp_e, Cn_e, 3);

% Initialize the graph with an empty node
G = digraph([],[]);
n_nodes = height(G.Nodes);

% Build a new node with the needed properties and insert in graph
% As we are initializing the tree, here Cont_h = false and Cp_h and Cn_h
% are empty
ID = n_nodes +1;
Cont_h = false;
dir0 = zeros(6,1); % direction of expansion is null here
dist0 = hom_dist(object_init.T, object_fin.T);
prev_pos0 = false;
NewNode = table( ID, ... 
    {object_init}', {robot}', {Cp_e}', {Cn_e}', {Cone}', ...
    Cont_h, {[]}', {[]}', {dir0}', dist0, prev_pos0, ...
    'VariableNames', ...
    {'ID' 'Object' 'Robot' 'Cp_e' 'Cn_e' 'Cone' 'Cont_h' 'Cp_h' 'Cn_h' ...
    'dir' 'dist' 'prev_pos'});
G = addnode(G,NewNode);

end

