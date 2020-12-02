% 
run(fullfile('..', 'tools', 'resolve_paths.m'))

T = eye(4);
T(1,4) = 5.5;
T(2,4) = 6.5;
T(3,4) = 1;
box_object = build_box(1,3,2,T);

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

axis_dim = [-5 5 0 10 0 10];
view_pose = [45.7, 50];

figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
plot_boxes(all_boxes, true);
xlabel('z');
ylabel('x');
zlabel('y');
% axis equal

legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k')],...
    {'Environment','Initial Position', 'Goal Position'},...
    'Location','northeast','FontSize',16);
axis(axis_dim);
view(view_pose(1), view_pose(2));