%
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

dm_to_m = 0.5e-1;

T = eye(4);
T(1,4) = 7*dm_to_m;
T(2,4) = 1.5*dm_to_m;
T(3,4) = 1.75*dm_to_m;
box_object = build_box(2*dm_to_m,1*dm_to_m,3*dm_to_m,T);

T =  troty(pi/2);
T(1,4) = 7.75*dm_to_m;
T(2,4) = 5*dm_to_m;
T(3,4) = 4*dm_to_m;
target_position = box_object;
target_position.T = T;

azim = 75.5;
elev = 11;
axis_range = [-5 5 -5 5 -1 6];

environment = {};
all_boxes = {box_object};