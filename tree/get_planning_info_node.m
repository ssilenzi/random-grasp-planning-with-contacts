function [arm_pose,gripper_pos,type,same_face,sym_cont] = ...
    get_planning_info_node(G,node_ID, prev_node_ID)
% GET PLANNING INFO NODE - Get all the necessary info for filling the
% panda_gripper_manipulation messages

% Output: geometry_msgs/Pose for arm and sensor_msgs/JointState for gripper

arm_pose = rosmessage('geometry_msgs/Pose');
gripper_pos = rosmessage('sensor_msgs/JointState');

% Get node info
node_i = G.Nodes(node_ID,:); % row corresponding to node_ID
node_prev = G.Nodes(prev_node_ID,:); % row corresponding to prev_node_ID
robot_i = node_i.Robot{1};
cont_h_n_i = node_i.Cn_h{1};
cont_h_n_prev = node_prev.Cn_h{1};

% Getting needed pose data
hom_hand_i = robot_i.T_all(:,:,9); % panda_link8
pos_hand_i = hom_hand_i(1:3,4);
rot_hand_i = hom_hand_i(1:3,1:3);
quat_hand_i = rotm2quat(rot_hand_i);
fing_pos_i = robot_i.q(end);

% Get previous edge type info
ind = findedge(G, prev_node_ID, node_ID);
type_i = G.Edges(ind,2).Type{1};
type_i = type_i(1:3);

% Checking if same face contact (according to edge)
same_face_i = false;
sym_cont_i = false;
if strcmp(type_i, 'rel')
    cont_h_n =  cont_h_n_prev;
else
    cont_h_n =  cont_h_n_i;
end
if ~isempty(cont_h_n)
    if (cont_h_n(1,:) == cont_h_n(2,:))
        same_face_i = true;
    end
    if (cont_h_n(1,:) == -cont_h_n(2,:))
        sym_cont_i = true;
    end
end

% Filling in messages
gripper_pos.Position = fing_pos_i; % - 0.002; shelf book
arm_pose.Position.X = pos_hand_i(1);
arm_pose.Position.Y = pos_hand_i(2);
arm_pose.Position.Z = pos_hand_i(3);
% % Ad hoc for shelf box
% if node_ID == 2
%     arm_pose.Position.X = arm_pose.Position.X - 0.01;
%     gripper_pos.Position = fing_pos_i + 0.005;
% end
% if node_ID == 5
%     arm_pose.Position.Z = arm_pose.Position.Z - 0.2;
% end
% if node_ID == 6
%     arm_pose.Position.X = arm_pose.Position.X + 0.02;
%     arm_pose.Position.Y = arm_pose.Position.Y - 0.01;
% end
% if node_ID == 25
%     arm_pose.Position.Z = arm_pose.Position.Z - 0.015;
% end
% % End
% % Ad hoc for shelf book
% if node_ID == 3
%     arm_pose.Position.Z = arm_pose.Position.Z - 0.015;
%     gripper_pos.Position = fing_pos_i - 0.002;
% end
% if node_ID == 4
%     arm_pose.Position.Z = arm_pose.Position.Z - 0.15;
% end
% if node_ID == 5
%     arm_pose.Position.X = arm_pose.Position.X + 0.022;
%     arm_pose.Position.Z = arm_pose.Position.Z - 0.015;
% end
% % End
% % Ad hoc for cluttered
% arm_pose.Position.Z = arm_pose.Position.Z - 0.055;
% if node_ID == 4
%     arm_pose.Position.Y = arm_pose.Position.Y + 0.045;
%     arm_pose.Position.Z = arm_pose.Position.Z + 0.035;
% end
% % End
% Ad hoc for sliding
arm_pose.Position.Z = arm_pose.Position.Z - 0.055;
if node_ID == 6
    arm_pose.Position.Z = arm_pose.Position.Z + 0.01;
end
if node_ID > 7
    arm_pose.Position.Y = arm_pose.Position.Y + (node_ID/18)*0.15;
end
if node_ID == 18
    arm_pose.Position.Z = arm_pose.Position.Z - 0.0075;
end
% End
arm_pose.Orientation.X = quat_hand_i(2); % x
arm_pose.Orientation.Y = quat_hand_i(3); % y
arm_pose.Orientation.Z = quat_hand_i(4); % z
arm_pose.Orientation.W = quat_hand_i(1); % w

type = type_i;
same_face = same_face_i;
sym_cont = sym_cont_i;

end

