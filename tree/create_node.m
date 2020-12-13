function [node_f] = create_node(ID_f,box_f,robot_f,Cp_e_f,Cn_e_f,Cone_f, ...
    Cont_h_f,Cp_h_f,Cn_h_f,dir_f,prev_pos_f)
% CREATE NODE 
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

% Adding the newly created node
node_f = table( ID_f, ...
    {box_f}', {robot_f}', {Cp_e_f}', {Cn_e_f}', {Cone_f}', ...
    Cont_h_f, {Cp_h_f}', {Cn_h_f}', {dir_f}', dist_f, prev_pos_f, ...
    'VariableNames', ...
    {'ID' 'Object' 'Robot' 'Cp_e' 'Cn_e' 'Cone' 'Cont_h' ...
    'Cp_h' 'Cn_h' 'dir' 'dist' 'prev_pos'});

end

