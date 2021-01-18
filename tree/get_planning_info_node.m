function [arm_pose,gripper_pos,type,same_face] = get_planning_info_node(G,node_ID, prev_node_ID)
% GET PLANNING INFO NODE - Get all the necessary info for filling the
% panda_gripper_manipulation messages

% Output: geometry_msgs/Pose for arm and sensor_msgs/JointState for gripper

arm_pose = rosmessage('geometry_msgs/Pose');
gripper_pos = rosmessage('sensor_msgs/JointState');

% Get node info
node_i = G.Nodes(node_ID,:); % row corresponding to node_ID
robot_i = node_i.Robot{1};
cont_h_n_i = node_i.Cn_h{1};

% Checking if same face contact
same_face_i = false;
if ~isempty(cont_h_n_i)
    if (cont_h_n_i(1,:) == cont_h_n_i(2,:))
        same_face_i = true;
    end
end

% Getting needed pose data
hom_hand_i = robot_i.T_all(:,:,9);
pos_hand_i = hom_hand_i(1:3,4);
rot_hand_i = hom_hand_i(1:3,1:3);
quat_hand_i = rotm2quat(rot_hand_i);
fing_pos_i = robot_i.q(end);

% Get previous edge type info
ind = findedge(G, prev_node_ID, node_ID);
type_i = G.Edges(ind,2).Type{1};
type_i = type_i(1:3);

% Filling in messages
arm_pose.Position.X = pos_hand_i(1);
arm_pose.Position.Y = pos_hand_i(2);
arm_pose.Position.Z = pos_hand_i(3);
arm_pose.Orientation.X = quat_hand_i(2); % x
arm_pose.Orientation.Y = quat_hand_i(3); % y
arm_pose.Orientation.Z = quat_hand_i(4); % z
arm_pose.Orientation.W = quat_hand_i(1); % w
gripper_pos.Position = fing_pos_i;

type = type_i;
same_face = same_face_i;

end

