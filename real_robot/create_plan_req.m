function planreq = create_plan_req(plan_client, ...
    start_joint_msg, des_pose, type)
% CREATE PLAN REQ
% if des_pose -> des_pose = [px py pz, qx qy qz qw]
geom_fin_msg = rosmessage('geometry_msgs/Pose');
geom_fin_msg.Position.X = des_pose(1);
geom_fin_msg.Position.Y = des_pose(2);
geom_fin_msg.Position.Z = des_pose(3);
geom_fin_msg.Orientation.X = des_pose(4);
geom_fin_msg.Orientation.Y = des_pose(5);
geom_fin_msg.Orientation.Z = des_pose(6);
geom_fin_msg.Orientation.W = des_pose(7);

gripper_pos = rosmessage('sensor_msgs/JointState');
gripper_pos.Position = 0.04;

% Creating and filling up request
planreq = rosmessage(plan_client);
planreq.Waypoints = geom_fin_msg;
planreq.FingerStates = gripper_pos;
planreq.StartArmState = start_joint_msg; % empty for now
planreq.Transition = type;
planreq.SameFaceContacts = false;
planreq.SymContacts = true;

end

