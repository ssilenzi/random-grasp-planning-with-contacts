function [ID_out,box_out,robot_out,Cp_e_out,Cn_e_out,Cone_out, ...
    Cont_h_out,Cp_h_out,Cn_h_out,dir_out,prev_pos_out] = get_node_properties(node_in)

% GETS THE PROPERTIES OF A NODE
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

% Get the properties of the input node
ID_out = node_in.ID;
box_out = node_in.Object{1};
robot_out = node_in.Robot{1};
Cp_e_out = node_in.Cp_e{1};
Cn_e_out = node_in.Cn_e{1};
Cone_out = node_in.Cone{1};
Cont_h_out = node_in.Cont_h; % not a cell as only true or false
Cp_h_out = node_in.Cp_h{1};
Cn_h_out = node_in.Cn_h{1};
dir_out = node_in.dir{1};
prev_pos_out = node_in.prev_pos;

end

