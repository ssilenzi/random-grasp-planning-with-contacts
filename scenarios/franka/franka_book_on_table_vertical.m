%
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

dm_to_m = 0.5e-1;

T = eye(4);
T(1,4) = 7*dm_to_m;
T(2,4) = 1.5*dm_to_m;
T(3,4) = 1.75*dm_to_m;
box_object = build_box(2*dm_to_m,1*dm_to_m,3*dm_to_m,T);
box_object_col = collisionBox(2*dm_to_m,1*dm_to_m,3*dm_to_m); % coll box
box_object_col.Pose = T;

T =  troty(pi/2);
T(1,4) = 7.75*dm_to_m;
T(2,4) = 5*dm_to_m;
T(3,4) = 4*dm_to_m;
target_position = box_object;
target_position.T = T;

T = eye(4);
T(1,4) = 6.5*dm_to_m;
T(2,4) = -0.25*dm_to_m;
T(3,4) = 0;
box_table = build_box(10*dm_to_m,10*dm_to_m,0.5*dm_to_m, T);
box_table_col = collisionBox(10*dm_to_m,10*dm_to_m,0.5*dm_to_m); % coll box
box_table_col.Pose = T;

azim = 45.7;
elev = 50;
axis_range = [-5 5 -5 5 -1 6];

environment = {box_table};
all_boxes = {box_table, box_object};
coll_boxes = {box_table_col, box_object_col};