function boxes_to_planning_scene(boxes_arr, scale)
% ENV TO PLANNING SCENE - publishing all the input boxes array to planning
% scene of moveit
% Similar to sdf_to_planning_scene.cpp in panda_gripper_manipulation
% A scale in [0, 1] is used to scale the boxes so that moveit does not get
% stuck while planning (less conservative planning scene)

% Publisher and msgs
ps_pub = rospublisher('/planning_scene', 'moveit_msgs/PlanningScene');

while ps_pub.NumSubscribers < 1
    disp('Waiting for subscribers to the planning scene!');
    pause(1);
end

planning_scene = rosmessage('moveit_msgs/PlanningScene');
object = rosmessage('moveit_msgs/CollisionObject');
pose = rosmessage('geometry_msgs/Pose');
primitive = rosmessage('shape_msgs/SolidPrimitive');

% Filling up the main fields of the collision box
object.Header.FrameId = 'world';
pose.Orientation.W = 1.0;
primitive.Type = primitive.BOX;
object.Primitives(end+1) = primitive;
object.PrimitivePoses(end+1) = pose;

% An attach operation requires an ADD
object.Operation = object.ADD;

% Put the object in the environment
planning_scene.IsDiff = true;
planning_scene.RobotState.IsDiff = true;

% Getting one by one the boxes and publishing to planning scene
counter = 1;
for i = 1:length(boxes_arr)
    
    % Setting the box to the ith element in boxes array
    name = ['box_', num2str(counter)];
    box_i = boxes_arr{i};
    dim = [box_i.l, box_i.w, box_i.h];
    pos = box_i.T(1:3,4);
    quat = rotm2quat(box_i.T(1:3,1:3));
    set_object_properties(name, dim, pos, quat, object, scale);
    
    % Filling into planning scene message and publishing
    planning_scene.World.CollisionObjects = [];
    planning_scene.World.CollisionObjects(1) = object;
    disp(['Publishing box no.', num2str(counter)]);
    send(ps_pub,planning_scene);
    
    counter = counter + 1;
    
end

end

function set_object_properties(name, dim, pos, quat, object, scale)
    % pos = [x y z], quat = [w x y z], dim = [l w h], name = any name

    % Setting the name
    object.Id = name;
    
    % Setting the location
    object.PrimitivePoses(end).Position.X = pos(1);
    object.PrimitivePoses(end).Position.Y = pos(2);
    object.PrimitivePoses(end).Position.Z = pos(3);
    object.PrimitivePoses(end).Orientation.X = quat(2);
    object.PrimitivePoses(end).Orientation.Y = quat(3);
    object.PrimitivePoses(end).Orientation.Z = quat(4);
   	object.PrimitivePoses(end).Orientation.W = quat(1);
    
    % Setting the dimension
    object.Primitives(end).Dimensions(1) = scale*dim(1);
    object.Primitives(end).Dimensions(2) = scale*dim(2);
    object.Primitives(end).Dimensions(3) = scale*dim(3);
    
end

