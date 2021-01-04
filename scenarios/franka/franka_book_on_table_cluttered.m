%
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

dm_to_m = 0.5e-1;

T = eye(4);
T(3,4) = 0.75*dm_to_m;
T(2,4) = 0.5*dm_to_m;
T(1,4) = 6.5*dm_to_m;
box_object = build_box(2*dm_to_m,3*dm_to_m,1*dm_to_m,T);
box_object_col = collisionBox(2*dm_to_m,3*dm_to_m,1*dm_to_m); % coll box
box_object_col.Pose = T;

T = eye(4);
T =  troty(pi/2);
T(3,4) = 8*dm_to_m;
T(2,4) = 5*dm_to_m;
T(1,4) = 6.75*dm_to_m;
target_position = box_object;
target_position.T = T;

T = eye(4);
T(3,4) = 0*dm_to_m;
T(2,4) = 0*dm_to_m;
T(1,4) = 6.5*dm_to_m;
box_table = build_box(10*dm_to_m,10*dm_to_m,0.5*dm_to_m,T);
box_table_col = collisionBox(10*dm_to_m,10*dm_to_m,0.5*dm_to_m); % coll box
box_table_col.Pose = T;

T = eye(4);
T(3,4) = 0.75*dm_to_m;
T(2,4) = 0.5*dm_to_m;
T(1,4) = 8.0*dm_to_m;
box_left = build_box(1*dm_to_m,3*dm_to_m,1*dm_to_m,T);
box_left_col = collisionBox(1*dm_to_m,3*dm_to_m,1*dm_to_m); % coll box
box_left_col.Pose = T;

T = eye(4);
T(3,4) = 0.75*dm_to_m;
T(2,4) = 0.5*dm_to_m;
T(1,4) = 5.0*dm_to_m;
box_right = build_box(1*dm_to_m,3*dm_to_m,1*dm_to_m,T);
box_right_col = collisionBox(1*dm_to_m,3*dm_to_m,1*dm_to_m); % coll box
box_right_col.Pose = T;

T(3,4) = 1.75*dm_to_m;
T(2,4) = 0.5*dm_to_m;
T(1,4) = 6.5*dm_to_m;
box_top = build_box(4*dm_to_m,3*dm_to_m,1*dm_to_m,T);
box_top_col = collisionBox(4*dm_to_m,3*dm_to_m,1*dm_to_m); % coll box
box_top_col.Pose = T;

azim = 45.7;
elev = 50;
axis_range = [-5 5 0 10 0 10];

environment = {box_table, box_left, box_right, box_top};
all_boxes = {box_table, box_left, box_right, box_top, box_object};
coll_boxes = {box_table_col, box_left_col, box_right_col, ...
    box_top_col, box_object_col};
