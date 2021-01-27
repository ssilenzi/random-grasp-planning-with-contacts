function success = retract_franka(plan_client, control_client, ...
    start_joint_msg, dpose, des_pose)
% RETRACT FRANKA
% if dpose -> add to current transform
% if des_pose -> des_pose = [px py pz, qx qy qz qw]

% Getting the tree and the transform
tftree = rostf;
waitForTransform(tftree,'/world','/panda_link8',5);
curr_tran = getTransform(tftree, 'world', 'panda_link8');

% Creating and filling the geometry msg pose
geom_fin_msg = rosmessage('geometry_msgs/Pose');
if ~isempty(dpose)
geom_fin_msg.Position.X = curr_tran.Transform.Translation.X + dpose(1);
geom_fin_msg.Position.Y = curr_tran.Transform.Translation.Y + dpose(2);
geom_fin_msg.Position.Z = curr_tran.Transform.Translation.Z + dpose(3); % higher
geom_fin_msg.Orientation = curr_tran.Transform.Rotation; 
end

if ~isempty(des_pose)
    geom_fin_msg.Position.X = des_pose(1);
    geom_fin_msg.Position.Y = des_pose(2);
    geom_fin_msg.Position.Z = des_pose(3);
    geom_fin_msg.Orientation.X = des_pose(4);
    geom_fin_msg.Orientation.Y = des_pose(5);
    geom_fin_msg.Orientation.Z = des_pose(6);
    geom_fin_msg.Orientation.W = des_pose(7);
end

gripper_pos = rosmessage('sensor_msgs/JointState');
gripper_pos.Position = 0.04;

% Creating and filling up request
planreq = rosmessage(plan_client);
planreq.Waypoints = geom_fin_msg;
planreq.FingerStates = gripper_pos;
planreq.StartArmState = start_joint_msg; % empty for now
planreq.Transition = 'rel';
planreq.SameFaceContacts = false;
planreq.SymContacts = true;

% Calling service planning
planresp = call(plan_client,planreq,'Timeout',100);

% If good, calling robot control
if planresp.Answer
    
    % Creating and filling up control message
    controlreq = rosmessage(control_client);
    controlreq.ComputedTrajectory = planresp.ComputedTrajectory;
    controlreq.ComputedGripperPosition = planresp.ComputedGripperPosition;
    controlreq.Transition = planresp.Transition;
    controlreq.SameFaceContacts = planresp.SameFaceContacts;
    controlreq.SymContacts = planresp.SymContacts;
    
    % Calling service control
    controlresp = call(control_client,controlreq,'Timeout',100);
    
end

success = controlresp.Answer;

end

