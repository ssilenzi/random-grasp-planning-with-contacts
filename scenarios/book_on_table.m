%
run(fullfile('..', 'tools', 'resolve_paths.m'))

T = eye(4);
T(1:3,1:3) = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];
T(1,4) = -4*0;
T(2,4) = 0.5;
T(3,4) = -3.5*0;
box_object = build_box(2,1,3,T);

T =  trotz(pi/2);
T(1,4) = 3;
T(2,4) = 1.0;
T(3,4) = 3;

% T = eye(4);
% T =  trotz(pi/2);
% T(1,4) = 8;
% T(2,4) = 5;
% T(3,4) = 1.25;
target_position = box_object;
target_position.T = T;

T = eye(4);
T(1,4) = 0;
T(2,4) = -0.25;
T(3,4) = 0;
box_table = build_box(10,0.5,10, T);

environment = {box_table};
all_boxes = {box_table, box_object};
figure('Color',[1 1 1], 'pos',[10 10 1000 1000]);
plot_boxes(all_boxes, true);
plot_box(target_position.l, target_position.w, target_position.h, ...
    target_position.T, [0 0 0], true)
xlabel('z');
ylabel('x');
zlabel('y');
axis equal
view(45.7, 50);
legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k')],...
    {'Environment','Initial Position', 'Goal Position'},...
    'Location','northeast');