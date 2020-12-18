%
run(fullfile('..', 'tools', 'resolve_paths.m'))

T = eye(4);
T(1,4) = 0;
T(2,4) = 1.5;
T(3,4) = 0;
box_object = build_box(1,3,2,T);

T = eye(4);
T =  trotz(pi/2);
T(1,4) = 8;
T(2,4) = 5;
T(3,4) = 1.25;
target_position = box_object;
target_position.T = T;

azim = 45.7;
elev = 50;
axis_range = [-5 5 -5 5 -1 6];

environment = {};
all_boxes = {box_object};
figure('Color',[1 1 1], 'pos',[10 10 1000 1000]);
plot_boxes(all_boxes, true);
plot_box(target_position.l, target_position.w, target_position.h, ...
    target_position.T, [0 0 0], true)
xlabel('z');
ylabel('x');
zlabel('y');
axis equal
view(azim, elev);
legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k')],...
    {'Environment','Initial Position', 'Goal Position'},...
    'Location','northeast');