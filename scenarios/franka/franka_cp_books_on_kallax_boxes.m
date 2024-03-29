%
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

dm_to_m = 1e-1;

% Table
T = eye(4);
T(1,4) = 4.715*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = -0.125*dm_to_m;
box_table = build_box(7.3*dm_to_m,7*dm_to_m,0.25*dm_to_m, T);

% Base of Kallax
T = eye(4);
T(1,4) = 8.365*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 0.175*dm_to_m;
box_base_kallax = build_box(3.9*dm_to_m,7.7*dm_to_m,0.35*dm_to_m, T);

% Middle shelf of Kallax
T = eye(4);
T(1,4) = 8.365*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 3.9*dm_to_m;
box_middle_hor_kallax = build_box(3.9*dm_to_m,7*dm_to_m,0.15*dm_to_m, T);

% Vertical middle of Kallax
T = eye(4);
T(1,4) = 8.365*dm_to_m;
T(2,4) = 0*dm_to_m;
T(3,4) = 3.85*dm_to_m;
box_middle_ver_kallax = build_box(3.9*dm_to_m,0.2*dm_to_m,7*dm_to_m, T);

% Vertical left of Kallax
T = eye(4);
T(1,4) = 8.365*dm_to_m;
T(2,4) = -3.675*dm_to_m;
T(3,4) = 3.85*dm_to_m;
box_left_ver_kallax = build_box(3.9*dm_to_m,0.35*dm_to_m,7*dm_to_m, T);

% Vertical right of Kallax
T = eye(4);
T(1,4) = 8.365*dm_to_m;
T(2,4) = 3.675*dm_to_m;
T(3,4) = 3.85*dm_to_m;
box_right_ver_kallax = build_box(3.9*dm_to_m,0.35*dm_to_m,7*dm_to_m, T);

% FROM LEFT TO RIGHT THE BOOKS

% Books Left Left
T = eye(4);
T(1,4) = 8.365*dm_to_m;
T(2,4) = 2.775*dm_to_m;
T(3,4) = 5.575*dm_to_m;
books_ll = build_box(3.9*dm_to_m,1.45*dm_to_m,3.2*dm_to_m,T);

% Books Left Right
T = eye(4);
T(1,4) = 8.365*dm_to_m;
T(2,4) = 0.875*dm_to_m;
T(3,4) = 5.025*dm_to_m;
books_lr = build_box(3.9*dm_to_m,1.55*dm_to_m,2.1*dm_to_m,T);

% Books Left Middle
T = eye(4);
T(1,4) = 7.215*dm_to_m;
T(2,4) = 1.85*dm_to_m;
T(3,4) = 5.175*dm_to_m;
books_lm = build_box(1.6*dm_to_m,0.4*dm_to_m,2.4*dm_to_m,T);

% Boxes Right Right
T = eye(4);
T(1,4) = 8.365*dm_to_m;
T(2,4) = -2.75*dm_to_m;
T(3,4) = 5.275*dm_to_m;
boxes_rr = build_box(3.9*dm_to_m,1.5*dm_to_m,2.6*dm_to_m,T);

% Boxes Right Left
T = eye(4);
T(1,4) = 8.365*dm_to_m;
T(2,4) = -0.7*dm_to_m;
T(3,4) = 5.175*dm_to_m;
boxes_rl = build_box(3.9*dm_to_m,1.2*dm_to_m,2.4*dm_to_m,T);

% Object
T = eye(4);
T(1,4) = 7.215*dm_to_m;
T(2,4) = -1.65*dm_to_m;
T(3,4) = 5.275*dm_to_m;
box_object = build_box(1.6*dm_to_m,0.7*dm_to_m,2.6*dm_to_m,T);

% Pile of books
T = eye(4);
T(1,4) = 4.215*dm_to_m;
T(2,4) = 2.7*dm_to_m;
T(3,4) = 0.625*dm_to_m;
books_pile = build_box(2.4*dm_to_m,1.6*dm_to_m,1.25*dm_to_m,T);

% Target
T =  trotx(pi/2);
T(1,4) = 4.215*dm_to_m;
T(2,4) = 3*dm_to_m;
T(3,4) = 1.45*dm_to_m;
target_position = build_box(1.6*dm_to_m,0.4*dm_to_m,2.4*dm_to_m,T);


azim = -20;
elev = 20;
axis_range = [-5 5 -5 5 -1 6];

environment = {box_table, box_base_kallax, ...
    box_middle_hor_kallax, box_middle_ver_kallax, ...
    box_left_ver_kallax, box_right_ver_kallax, ...
    books_ll, books_lr, books_lm, ...
    boxes_rl, boxes_rr, books_pile};
all_boxes = {box_table, box_base_kallax, ...
    box_middle_hor_kallax, box_middle_ver_kallax, ...
    box_left_ver_kallax, box_right_ver_kallax, ...
    books_ll, books_lr, books_lm, ...
    boxes_rl, boxes_rr, books_pile, box_object};   