function [ID_f,box_f,robot_f,Cp_e_f,Cn_e_f,Cone_f, ...
    Cont_h_f,Cp_h_f,Cn_h_f,dir_f,dist_f,prev_pos_f] = ...
    copy_node_properties(ID_s,box_s,robot_s,Cp_e_s,Cn_e_s,Cone_s, ...
    Cont_h_s,Cp_h_s,Cn_h_s,dir_s,dist_s,prev_pos_s)

% COPY NODE PROPERTIES

ID_f = ID_s;
box_f = box_s;
robot_f = copy(robot_s);
Cp_e_f = Cp_e_s;
Cn_e_f = Cn_e_s;
Cone_f = Cone_s;
Cont_h_f = Cont_h_s; % this one is false but will become true
Cp_h_f = Cp_h_s;
Cn_h_f = Cn_h_s;
dir_f = dir_s;
dist_f = dist_s;
prev_pos_f = prev_pos_s;

end

