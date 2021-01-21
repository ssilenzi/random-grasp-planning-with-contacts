%
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

dm_to_m = 1e-1;

% Table
T = eye(4);
T(1,4) = 4.65*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = -0.125*dm_to_m;
box_table = build_box(7.3*dm_to_m,7*dm_to_m,0.25*dm_to_m, T);

% Base of Kallax
T = eye(4);
T(1,4) = 8.3*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 0.175*dm_to_m;
box_base_kallax = build_box(3.9*dm_to_m,7.7*dm_to_m,0.35*dm_to_m, T);

% Middle shelf of Kallax
T = eye(4);
T(1,4) = 8.3*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 3.85*dm_to_m;
box_middle_hor_kallax = build_box(3.9*dm_to_m,7*dm_to_m,0.15*dm_to_m, T);

% Vertical middle of Kallax
T = eye(4);
T(1,4) = 8.3*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 3.85*dm_to_m;
box_middle_ver_kallax = build_box(3.9*dm_to_m,0.15*dm_to_m,7*dm_to_m, T);

% Vertical left of Kallax
T = eye(4);
T(1,4) = 8.3*dm_to_m;
T(2,4) = -3.675*dm_to_m;
T(3,4) = 3.85*dm_to_m;
box_left_ver_kallax = build_box(3.9*dm_to_m,0.35*dm_to_m,7*dm_to_m, T);

% Vertical right of Kallax
T = eye(4);
T(1,4) = 8.3*dm_to_m;
T(2,4) = 3.675*dm_to_m;
T(3,4) = 3.85*dm_to_m;
box_right_ver_kallax = build_box(3.9*dm_to_m,0.35*dm_to_m,7*dm_to_m, T);

% FROM LEFT TO RIGHT THE BOOKS

% Book 0
T = eye(4);
T(1,4) = 5.5*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 1.275*dm_to_m;
box_object = build_box(2.4*dm_to_m,0.7*dm_to_m,3*dm_to_m,T);

% Book 0
T = eye(4);
T(1,4) = 8.3*dm_to_m;
T(2,4) = 3.15*dm_to_m;
T(3,4) = 5.4*dm_to_m;
book_0 = build_box(3.9*dm_to_m,0.7*dm_to_m,3*dm_to_m,T);

% Target
T = eye(4);
T(1,4) = 5.5*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 1.275*dm_to_m;
target_position = build_box(1.3*dm_to_m,0.55*dm_to_m,2.55*dm_to_m,T);


azim = 75.5;
elev = 11;
axis_range = [-5 5 -5 5 -1 6];

environment = {box_table, box_base_kallax, ...
    box_middle_hor_kallax, box_middle_ver_kallax, ...
    box_left_ver_kallax, box_right_ver_kallax, ...
    book_0};
all_boxes = {box_table, box_base_kallax, ...
    box_middle_hor_kallax, box_middle_ver_kallax, ...
    box_left_ver_kallax, box_right_ver_kallax, ...
    book_0, box_object};   