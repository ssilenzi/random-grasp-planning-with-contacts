function [node_out] = move_stuff_in_node(node_in, d_pose, environment)
% MOVE STUFF IN NODE

% Getting the props
[ID_in,box_in,robot_in,Cp_e_in,Cn_e_in,Cone_in, ...
    Cont_h_in,Cp_h_in,Cn_h_in,dir_in,dist_in,prev_pos_in] = ...
    get_node_properties(node_in);

% Plotting for safety
figure('units','normalized','outerposition',[0 0 1 1]);
axis equal;
axis off;
view(50, 30);
grid off
plot_boxes(environment, true, false);
plot_box(box_in.l, box_in.w, box_in.h, box_in.T, [0 0 1], true);
plot_contacts(Cp_h_in, Cn_h_in, [0 1 0]);
plot_contacts(Cp_e_in, Cn_e_in, [0 1 0]);
rob_handle = robot_in.plot();

% Moving object
box_out = twist_moves_object(box_in, d_pose);

% Finding the moved contact points and normals and new robot config
Hom_d_pose = twistexp(d_pose); % homogeneous trans. corresponding to obj twist
Cp_h_out = transform_points(Cp_h_in, Hom_d_pose);      % transforming points
Cn_h_out = transform_vectors(Cn_h_in, Hom_d_pose);     % transforming normals

% Wanted wrist transform
robot_out = copy(robot_in);
wrist0 = robot_out.get_forward_kinematics();
wrist0 = wrist0(1:4,4,3); % previous wrist position
wrist1 = Hom_d_pose * wrist0;
wrist1 = wrist1(1:3);

% Minor fix
robot_out.sig_act = robot_out.q;

% Moving the robot with the wanted wrist transform
if ~isempty(Cp_h_out)
    [robot_out, success] = move_robot_to_points_and_wrist(robot_out,Cp_h_out,wrist1);
    if ~success
        warning('The hand was not moved correctly!');
    end
end

% Creating new node stuff
[Cp_e_out, Cn_e_out] = get_contacts(environment, box_out, box_out.T);
Cone_out = pfc_analysis(Cp_e_out, Cn_e_out, 3);

% Plotting for safety
figure('units','normalized','outerposition',[0 0 1 1]);
axis equal;
axis off;
view(50, 30);
grid off
plot_boxes(environment, true, false);
plot_box(box_out.l, box_out.w, box_out.h, box_out.T, [0 0 1], true);
plot_contacts(Cp_h_out, Cn_h_out, [0 1 0]);
plot_contacts(Cp_e_out, Cn_e_out, [0 1 0]);
rob_handle = robot_out.plot();

% Creating output node
[node_out] = create_node(ID_in,box_out,robot_out,Cp_e_out,Cn_e_out,Cone_out, ...
    Cont_h_in,Cp_h_out,Cn_h_out,dir_in,dist_in,prev_pos_in);

end

