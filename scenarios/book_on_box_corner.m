%
run(fullfile('..', 'tools', 'resolve_paths.m'))

T = eye(4);
T(1,4) = -4;
T(2,4) = 0.5;
T(3,4) = -3.5;
box_object = build_box(2,1,3,T);

T = eye(4);
T =  trotz(pi/2)*trotx(pi/4);
T(1,4) = 0;
T(2,4) = 1;
T(3,4) = 0.5;
target_position = box_object;
target_position.T = T;

T = eye(4);
T(1,4) = 0;
T(2,4) = -0.25;
T(3,4) = 0;
box_table = build_box(10,0.5,10, T);

T = eye(4);
T(1,4) = 0;
T(2,4) = 4.75;
T(3,4) = -5.25;
box_wall1 = build_box(10,10,0.5, T);

T = eye(4);
T(1,4) = -5.25;
T(2,4) = 4.75;
T(3,4) = 0;
box_wall2 = build_box(0.5,10,10, T);

azim = 123.3;
elev = 46;
axis_range = [-6 6 -6 6 -1 10];

environment = {box_table, box_wall1, box_wall2,};
all_boxes = {box_table, box_wall1, box_wall2, box_object};
figure('Color',[1 1 1], 'pos',[10 10 1000 1000]);
plot_boxes(all_boxes, true);
plot_box(target_position.l, target_position.w, target_position.h, ...
    target_position.T, [0 0 0], true)
xlabel('z');
ylabel('x');
zlabel('y');
axis equal
view(123.3, 46);
legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-g'),plot(NaN,NaN,'-k')],...
    {'Environment','Tree', 'Final Path', 'Goal Configuration'},...
    'Location','northeast','FontSize',16);