%% For testing hand functions for starting and ik

close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Load the environment and the box (both initial and final poses)
% run('book_on_shelf_no_target.m')
% run('book_on_table.m')
run('book_on_shelf_no_other_books.m')
% run('book_on_table_cluttered_no_target.m')
% run('book_on_box_corner_no_target.m')

axis([-10 10 -15 15 -15 15]); % Change the axis and view
view(50, 30);
legend off;

% Get object position as row
Co = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp, Cn] = get_contacts(environment, box_object, box_object.T);
% plot_contacts(Cp,Cn);

% Getting the cone and moving the object
Cone = pfc_analysis(Cp, Cn, 3);
dt = 0.5;

% for i=1:size(Cone,2)
%     figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
%     str = sprintf('Twist: [%f %f %f %f %f %f]',Cone(3,i),Cone(1,i),...
%         Cone(2,i),Cone(6,i),Cone(4,i),Cone(5,i));
%     title(str)
%     axis([-10 10 -15 15 -15 15]);
%     axis equal;
%     view(50, 30);
%     grid off
%     boxn = twist_moves_object(box_object, Cone(:,i)*dt);
%     plot_boxes(all_boxes, true);
%     plot_box(boxn.l, boxn.w, boxn.h, boxn.T, [0 0 0], true);
%     xlabel('z');
%     ylabel('x');
%     zlabel('y');
% end

alpha = zeros(size(Cone,2),1); alpha(2) = 1; % pick an alpha
twist = Cone*alpha*dt; % define the twist to test
box_object = twist_moves_object(box_object, twist);
plot_box(box_object.l, box_object.w, box_object.h, ...
    box_object.T, [0 0 0], true)

% Get newcontacts with the environment and plot
[Cp, Cn] = get_contacts(environment, box_object, box_object.T);
plot_contacts(Cp,Cn);
