% 
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

dm_to_m = 1e-1;

T = eye(4);
T(3,4) = 3.25*dm_to_m;
T(2,4) = 0*dm_to_m;
T(1,4) = 6.75*dm_to_m;
box_object = build_box(1*dm_to_m,0.5*dm_to_m,1.5*dm_to_m,T);

T = eye(4);
T(3,4) = 3.25*dm_to_m;
T(2,4) = 1.25*dm_to_m;
T(1,4) = 5*dm_to_m;
target_position = build_box(1*dm_to_m,0.5*dm_to_m,1.5*dm_to_m,T);

T = eye(4);
T(3,4) = 2.375*dm_to_m;
T(2,4) = 0.375*dm_to_m;
T(1,4) = 6.5*dm_to_m;
box_shelf = build_box(1.5*dm_to_m,5*dm_to_m,0.25*dm_to_m,T);

T = eye(4);
T(3,4) = 3.5*dm_to_m;
T(2,4) = 1.5*dm_to_m;
T(1,4) = 6.75*dm_to_m;
box_left = build_box(1*dm_to_m,2.5*dm_to_m,2*dm_to_m,T);

T = eye(4);
T(3,4) = 3.5*dm_to_m;
T(2,4) = -0.75*dm_to_m;
T(1,4) = 6.75*dm_to_m;
box_right = build_box(1*dm_to_m,1*dm_to_m,2*dm_to_m,T);

T = eye(4);
T(3,4) = 2.5*dm_to_m;
T(2,4) = 0.5*dm_to_m;
T(1,4) = 7.5*dm_to_m;
box_wall = build_box(0.5*dm_to_m,5*dm_to_m,5*dm_to_m,T);

environment = {box_shelf, box_left, box_right, box_wall};
all_boxes = {box_shelf, box_left, box_right, box_wall, box_object};

azim = -33.5;
elev = 40;
axis_range = [-1 7 0 10 0 10];
