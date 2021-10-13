%
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

dm_to_m = 1e-1;

T = eye(4);
T(1,4) = 4.5*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 3.25*dm_to_m;
box_object = build_box(0.5*dm_to_m,1.5*dm_to_m,1.5*dm_to_m,T);

T =  eye(4);
T(1,4) = 4*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 3.25*dm_to_m;
box_near = box_object;
box_near.T = T;

T =  eye(4);
T(1,4) = 5*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 3.25*dm_to_m;
box_far = box_object;
box_far.T = T;

T = eye(4);
T(1,4) = 4.5*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 4.25*dm_to_m;
box_above = build_box(1.5*dm_to_m,1.5*dm_to_m,0.5*dm_to_m,T);

T =  trotz(-pi/2);
T(1,4) = 6*dm_to_m;
T(2,4) = 2.5*dm_to_m;
T(3,4) = 3.25*dm_to_m;
target_position = box_object;
target_position.T = T;

T = eye(4);
T(1,4) = 4.65*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 2.375*dm_to_m;
box_table = build_box(7.3*dm_to_m,7*dm_to_m,0.25*dm_to_m, T);

azim = 75.5;
elev = 11;
axis_range = [-5 5 -5 5 -1 6];

environment = {box_table, box_near, box_far, box_above};
all_boxes = {box_table, box_near, box_far, box_above, box_object};