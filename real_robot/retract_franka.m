function success = retract_franka(plan_client, control_client, start_joint_msg)
% RETRACT FRANKA

% Getting the tree and the transform
tftree = rostf;
waitForTransform(tftree,'/world','/panda_link8',5);
curr_tran = getTransform(tftree, 'world', 'panda_link8');

% Creating and filling the geometry msg pose
geom_fin_msg = rosmessage('geometry_msgs/Pose');
geom_fin_msg.Position.X = curr_tran.Transform.Translation.X;
geom_fin_msg.Position.Y = curr_tran.Transform.Translation.Y;
geom_fin_msg.Position.Z = curr_tran.Transform.Translation.Z + 0.2; % higher
geom_fin_msg.Orientation = curr_tran.Transform.Rotation;

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

