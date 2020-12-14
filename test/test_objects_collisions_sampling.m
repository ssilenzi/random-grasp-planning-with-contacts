% Test the collision of two boxes with the environment
close all
clear
clc
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Load the environment and the boxes
fig = figure('Color',[1, 1, 1], 'Position',[10, 10, 1000, 1000]);
xlabel('z');
ylabel('x');
zlabel('y');
[handle, objects, environment] = build_objects_and_env();

% Check the collisions of the objects with the environment
plot_objects_collisions(fig, handle(2,:), objects, environment)

% Change the axis and view
axis([-4, 6, -5, 5, 0, 13])
axis equal
view([1, 1, 1])
zoom(1)
saveas(gcf, fullfile('..', 'figures', 'fig_sampling.png'))

function [handle, objects, environment] = build_objects_and_env()
T = [1,0,0,0;
    0,sin(30/180*pi),-cos(30/180*pi),0;
    0,cos(30/180*pi),sin(30/180*pi),0;
    0,0,0,1].';
T(1:3,4) = [8; 5; 2*cos(60/180*pi)] + T(1:3,1:3) * [0.5; 2; 1];
box_1 = build_box(1, 4, 2, T);

T = [1,0,0,0;
    0,sin(60/180*pi),-cos(60/180*pi),0;
    0,cos(60/180*pi),sin(60/180*pi),0;
    0,0,0,1].';
T(1:3,4) = [5; 5; 3*cos(60/180*pi)] + T(1:3,1:3) * [0.5; 1.5; 1];
box_2 = build_box(1, 3, 2, T);

T = eye(4);
T(1,4) = 2.5;
T(2,4) = 5.5;
T(3,4) = 2.5;
box_3 = build_box(1, 1, 3, T);

T = [cos(10/180*pi),sin(10/180*pi),0,0;
    -sin(10/180*pi),cos(10/180*pi),0,0;
    0,0,1,0;
    0,0,0,1].';
T(1:3,4) = [10+1; 0; 0.5] + T(1:3,1:3) * [1; 6; -1];
box_4 = build_box(2, 12, 2, T);

T = eye(4);
T(1,4) = 5;
T(2,4) = 4.75;
T(3,4) = 2;
box_shelf = build_box(10, 0.5, 4, T);

T = eye(4);
T(1,4) = 2.5;
T(2,4) = 7;
T(3,4) = 1;
box_left = build_box(5, 4, 2, T);

T = eye(4);
T(1,4) = 7;
T(2,4) = 7;
T(3,4) = 1;
box_right = build_box(2, 4, 2, T);

T = eye(4);
T(1,4) = 5;
T(2,4) = 5;
T(3,4) = -.5;
box_wall = build_box(10, 10, 1, T);

environment = {box_shelf, box_left, box_right, box_wall};
objects = {box_1, box_2, box_3, box_4};

handle(1,:) = plot_list_boxes(environment, [1, 0, 0]);
handle(2,:) = plot_list_boxes(objects, [0, 0, 1]);
end

function plot_objects_collisions(fig, handle, objects, environment)
tot_elapsed = 0;
for i = 1:size(objects,2)
    % this is the collision function!
    % a note: check_collisions_box can be also called with only 1 output
    my_tim = tic;
    [bool, coll_type] = check_collisions_box(objects{i}, environment, ...
        'sampling');
    curr_elapsed = toc(my_tim);
    fprintf('box_%d elapsed time: %f\n', i, curr_elapsed)
    tot_elapsed = tot_elapsed + curr_elapsed;
    % if there is a collision
    if bool == true
        fprintf('box_%d %s collision\n', i, coll_type)
        figure(fig)
        delete(handle{i})
        box = objects{i};
        handle{i} = plot_box(box.l, box.w, box.h, box.T, [0 1 0], true);
    else
        fprintf('box_%d no collision\n', i)
    end
end
fprintf('Total elapsed time: %f\n', tot_elapsed)
end

function handle = plot_list_boxes(list_boxes, RGB_color)
n_boxes = length(list_boxes);
handle = {};
for i = 1:n_boxes
    box = list_boxes{i};
    handle{i} = plot_box(box.l, box.w, box.h, box.T, RGB_color, true);
    hold on
end
end