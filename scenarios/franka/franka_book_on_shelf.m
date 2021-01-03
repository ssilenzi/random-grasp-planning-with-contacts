% 
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

dm_to_m = 0.5e-1;

T = eye(4);
T(3,4) = 6.5*dm_to_m;
T(2,4) = 0*dm_to_m;
T(1,4) = 13.5*dm_to_m;
box_object = build_box(2*dm_to_m,1*dm_to_m,3*dm_to_m,T,dm_to_m);
box_object_col = collisionBox(2*dm_to_m,1*dm_to_m,3*dm_to_m); % coll box
box_object_col.Pose = T;

T = eye(4);
T(3,4) = 6.5*dm_to_m;
T(2,4) = 2.5*dm_to_m;
T(1,4) = 10*dm_to_m;
target_position = build_box(2*dm_to_m,1*dm_to_m,3*dm_to_m,T,dm_to_m);

T = eye(4);
T(3,4) = 4.75*dm_to_m;
T(2,4) = 0.75*dm_to_m;
T(1,4) = 13*dm_to_m;
box_shelf = build_box(3*dm_to_m,10*dm_to_m,0.5*dm_to_m, T,dm_to_m);
box_shelf_col = collisionBox(3*dm_to_m,10*dm_to_m,0.5*dm_to_m); % coll box
box_shelf_col.Pose = T;

T = eye(4);
T(3,4) = 7*dm_to_m;
T(2,4) = 3*dm_to_m;
T(1,4) = 13.5*dm_to_m;
box_left = build_box(2*dm_to_m,5*dm_to_m,4*dm_to_m, T,dm_to_m);
box_left_col = collisionBox(2*dm_to_m,5*dm_to_m,4*dm_to_m); % coll box
box_left_col.Pose = T;

T = eye(4);
T(3,4) = 7*dm_to_m;
T(2,4) = -1.5*dm_to_m;
T(1,4) = 13.5*dm_to_m;
box_right = build_box(2*dm_to_m,2*dm_to_m,4*dm_to_m, T,dm_to_m);
box_right_col = collisionBox(2*dm_to_m,2*dm_to_m,4*dm_to_m); % coll box
box_right_col.Pose = T;

T = eye(4);
T(3,4) = 5*dm_to_m;
T(2,4) = 1*dm_to_m;
T(1,4) = 15*dm_to_m;
box_wall = build_box(1*dm_to_m,10*dm_to_m,10*dm_to_m, T,dm_to_m);
box_wall_col = collisionBox(1*dm_to_m,10*dm_to_m,10*dm_to_m); % coll box
box_wall_col.Pose = T;

environment = {box_shelf, box_left, box_right, box_wall};
all_boxes = {box_shelf, box_left, box_right, box_wall, box_object};
coll_boxes = {box_shelf_col, box_left_col, box_right_col, ...
    box_wall_col, box_object_col};

azim = -135.7;
elev = 30;
axis_range = [-1 7 0 10 0 10];
