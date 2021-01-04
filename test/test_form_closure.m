%% For testing hand functions for starting and ik

close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Load the environment and the box (both initial and final poses)
% run('book_on_table.m')
run('book_on_table_vertical.m')
% run('book_on_shelf_no_other_books.m')
% run('book_on_shelf_no_target.m')
% run('book_on_table_cluttered_no_target.m')
% run('book_on_box_corner_no_target.m')

axis([-15 15 -15 15 -15 15]); % Change the axis and view
axis equal;
view(50, 30);
legend off;

dt = 0.5;

% Get object position as row
Co = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp, Cn] = get_contacts(environment, box_object, box_object.T);
plot_contacts(Cp,Cn);

% Getting the cone and moving the object
tic
Cone = pfc_analysis(Cp, Cn, 3);
toc

% Selecting a combination vec. and moving the object
ind = randsample([1:size(Cone,2)],1);
alpha0 = zeros(size(Cone,2),1); alpha0(ind) = 1; %alpha0(5) = 1; % selecting a generator
tic
[success, box_obj1, twist01, d_pose01] = get_pose_from_cone(Cone, ...
    box_object, environment, dt, alpha0);
plot_box(box_obj1.l, box_obj1. w,box_obj1.h, box_obj1.T, [0 0 0], true);
toc

% Testing the found velocities with the correct grasp matrix
% G_bon = build_g(Cp);
% G_cor = build_g_cont(Cp,Co);
% 
% R_o = box_object.T(1:3,1:3).';
% 
% vel_cont_bon_1 = G_bon.'*d_pose01;
% vel_cont_cor_1 = G_cor.'*blkdiag(eye(3),R_o)*d_pose01;
% 
% vel_cont_bon_1 - vel_cont_cor_1
% 
% for i=1:size(Cone,2)
%     figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
%     str = sprintf('Twist: [%f %f %f %f %f %f]',Cone(3,i),Cone(1,i),...
%         Cone(2,i),Cone(6,i),Cone(4,i),Cone(5,i));
%     title(str)
%     axis([-15 15 -15 15 -15 15]);
%     axis equal;
%     view(50, 30);
%     grid off
%     boxn = twist_moves_object(box_object, Cone(:,i)*dt);
% %     alpha0 = zeros(size(Cone,2),1); alpha0(i) = 1;
% %     [~, boxn, ~, ~] = get_pose_from_cone(Cone, ...
% %     box_object, environment, dt, alpha0);
%     plot_boxes(all_boxes, true);
%     plot_box(boxn.l, boxn.w, boxn.h, boxn.T, [0 0 0], true);
%     xlabel('z');
%     ylabel('x');
%     zlabel('y');
% end

% alpha = zeros(size(Cone,2),1); alpha(2) = 1; % pick an alpha
% twist = Cone*alpha*dt; % define the twist to test
% box_object2 = twist_moves_object(box_object, twist);
% plot_box(box_object2.l, box_object2.w, box_object2.h, ...
%     box_object2.T, [0 0 0], true)

