% 
run(fullfile('..', 'tools', 'resolve_paths.m'))

T = eye(4);
T(1,4) = 5.5;
T(2,4) = 6.5;
T(3,4) = 1;
box_object = build_box(1,3,2,T);

T = eye(4);
T(1,4) = 2.5;
T(2,4) = 6.5;
T(3,4) = 3;
target_position = build_box(1,3,2,T);

T = eye(4);
T(1,4) = 5;
T(2,4) = 4.75;
T(3,4) = 1.5;
box_shelf = build_box(10,0.5,3, T);

T = eye(4);
T(1,4) = 5;
T(2,4) = 5;
T(3,4) = -.5;
box_wall = build_box(10,10,1, T);

environment = {box_shelf, box_wall};
all_boxes = {box_shelf, box_wall, box_object};

azim = 45.7;
elev = 50;
axis_range = [-5 5 0 10 0 10];

figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
plot_boxes(all_boxes, true);
plot_box(target_position.l, target_position.w, target_position.h, ...
    target_position.T, [0 0 0], true)
xlabel('z');
ylabel('x');
zlabel('y');

legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k')],...
    {'Environment','Initial Position', 'Goal Position'},...
    'Location','northeast','FontSize',16);
axis(axis_range);
view(azim, elev);