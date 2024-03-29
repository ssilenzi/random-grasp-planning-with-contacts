%
run(fullfile('..', 'tools', 'resolve_paths.m'))

T = eye(4);
T(1:3,1:3) = [0.9098, 0, 0.4151;
    0, 1.0000, 0;
    -0.4151, 0, 0.9098];
T(1,4) = -4*0;
T(2,4) = 0.5;
T(3,4) = -3.5*0;
box_object = build_box(2,1,3,T);

T = eye(4);
T(1,4) = 0;
T(2,4) = -0.25;
T(3,4) = 0;
box_table = build_box(10,0.5,10, T);

environment = {box_table};
all_boxes = {box_table, box_object};
figure('Color',[1 1 1], 'pos',[10 10 1000 1000]);
plot_boxes(all_boxes, true);
xlabel('z');
ylabel('x');
zlabel('y');
axis equal
view(45.7, 50);
legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k')],...
    {'Environment','Initial Position', 'Goal Position'},...
    'Location','northeast');