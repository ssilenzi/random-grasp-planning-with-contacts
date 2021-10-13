%
run(fullfile('..', 'tools', 'resolve_paths.m'))

% The dimensions are in meters.

% Table
T = eye(4);
T(1,4) = 0.4715;
T(2,4) = 0;
T(3,4) = -0.0125;
box_table = build_box(0.73,0.7,0.025, T);

% Base of Kallax
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = 0;
T(3,4) = 0.0175;
box_base_kallax = build_box(0.39,0.77,0.035, T);

% Middle shelf of Kallax
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = 0;
T(3,4) = 0.39;
box_middle_hor_kallax = build_box(0.39,0.7,0.015, T);

% Vertical middle of Kallax
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = 0;
T(3,4) = 0.385;
box_middle_ver_kallax = build_box(0.39,0.02,0.7, T);

% Vertical left of Kallax
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = -0.3675;
T(3,4) = 0.385;
box_left_ver_kallax = build_box(0.39,0.035,0.7, T);

% Vertical right of Kallax
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = 0.3675;
T(3,4) = 0.385;
box_right_ver_kallax = build_box(0.39,0.035,0.7, T);

% FROM LEFT TO RIGHT THE BOOKS

% Books Left Left 1
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = 0.31375;
T(3,4) = 0.5575;
book_ll1 = build_box(0.39,0.0725,0.32,T);

% Books Left Left 2
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = 0.24125;
T(3,4) = 0.5175;
book_ll2 = build_box(0.39,0.0725,0.24,T);

% Books Left Right
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = 0.0875;
T(3,4) = 0.5025;
book_lr = build_box(0.39,0.155,0.21,T);

% Object - Books Left Middle
T = eye(4);
T(1,4) = 0.7215;
T(2,4) = 0.185;
T(3,4) = 0.5175;
book_object = build_box(0.16,0.04,0.24,T);

% Boxes Right Right
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = -0.275;
T(3,4) = 0.5275;
book_rr = build_box(0.39,0.15,0.26,T);

% Boxes Right Left
T = eye(4);
T(1,4) = 0.8365;
T(2,4) = -0.07;
T(3,4) = 0.5175;
book_rl = build_box(0.39,0.12,0.24,T);

% Box Right Middle
T = eye(4);
T(1,4) = 0.7215;
T(2,4) = -0.165;
T(3,4) = 0.5275;
book_rm = build_box(0.16,0.07,0.26,T);

% Pile of books
T = eye(4);
T(1,4) = 0.4215;
T(2,4) = 0.27;
T(3,4) = 0.0625;
books_pile = build_box(0.24,0.16,0.125,T);

% Target
T =  trotx(pi/2);
T(1,4) = 0.4215;
T(2,4) = 0.3;
T(3,4) = 0.145;
target_position = build_box(0.16,0.04,0.24,T);

azim = -20;
elev = 20;
axis_range = [-5 5 -5 5 -1 6];

environment = {box_table, box_base_kallax, ...
    box_middle_hor_kallax, box_middle_ver_kallax, ...
    box_left_ver_kallax, box_right_ver_kallax, ...
    books_ll1, books_ll2, books_lr, box_rm, ...
    boxes_rl, boxes_rr, books_pile};
all_boxes = {box_table, box_base_kallax, ...
    box_middle_hor_kallax, box_middle_ver_kallax, ...
    box_left_ver_kallax, box_right_ver_kallax, ...
    books_ll1, books_ll2, books_lr, box_rm, ...
    boxes_rl, boxes_rr, books_pile, box_object};   
