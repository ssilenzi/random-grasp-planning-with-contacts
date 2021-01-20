%
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

dm_to_m = 1e-1;

T = eye(4);
T(1,4) = 4.5*dm_to_m;
T(2,4) = 1.75*dm_to_m;
T(3,4) = 0.27*dm_to_m;
box_object = build_box(1.3*dm_to_m,2.55*dm_to_m,0.55*dm_to_m,T);

T =  trotx(pi/2);
T(1,4) = 5*dm_to_m;
T(2,4) = -2.75*dm_to_m;
T(3,4) = 1.27*dm_to_m;
target_position = box_object;
target_position.T = T;

T = eye(4);
T(1,4) = 3.25*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = -0.13*dm_to_m;
box_table = build_box(10*dm_to_m,7*dm_to_m,0.25*dm_to_m, T);

azim = 75.5;
elev = 11;
axis_range = [-5 5 -5 5 -1 6];

environment = {box_table};
all_boxes = {box_table, box_object};