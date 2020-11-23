%% For testing hand functions for starting and ik

close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Load the environment and the box (both initial and final poses)
% run('book_on_table.m')
% run('book_on_shelf_no_other_books.m')
run('book_on_shelf_no_target.m')
% run('book_on_table_cluttered_no_target.m')

axis([-10 10 -15 15 -15 15]); % Change the axis and view
axis equal;
view(50, 30);
legend off;

% Get object position as row
Co = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp, Cn] = get_contacts(environment, box_object, box_object.T);
plot_contacts(Cp,Cn);

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

alpha = zeros(size(Cone,2),1); alpha(1) = 1; % pick an alpha
twist = Cone*alpha*dt; % define the twist to test
box_object2 = twist_moves_object(box_object, twist);
plot_box(box_object2.l, box_object2.w, box_object2.h, ...
    box_object2.T, [0 0 0], true)

% Get newcontacts with the environment and plot
[Cp, Cn] = get_contacts(environment, box_object, box_object.T);
plot_contacts(Cp,Cn);

% Get accessible faces
i_faces = get_free_box_faces(box_object, Cp, Cn);
plot_box_face(box_object, i_faces);

% Get random points on object faces
p = get_random_points_on_box_faces(box_object, i_faces, 2);
n = zeros(size(p));
for i=1:size(p,1)
    i_face = get_faces_from_points_indexes(box_object, p(i,:));
%     disp(i_face); disp(size(i_face));
    n(i,:) = box_object.face_normals(i_face,:);
end

% Transform random points to global reference system and plot
p_global = transform_points(p, box_object.T);
n_global = transform_vectors(n, box_object.T);
plot_contacts(p_global, n_global, [1 0 1]);

% Loading the hand
robot = load_gripper('hand_example');
q = robot.get_starting_config_george(p_global, n_global);
robot.set_config(q);

handle1 = robot.plot();

% Getting the current wrist pose
x_now = robot.get_forward_kinematics();
x_wrist = x_now(:,:,3);

% Setting the desired points and IK
xd(:,:,1) = [eye(3) p_global(1,1:3).'; [0 0 0 1]];
xd(:,:,2) = [eye(3) p_global(2,1:3).'; [0 0 0 1]];
xd(:,:,3) = x_wrist;
q_open_d = robot.q(7:8);

robot.compute_differential_inverse_kinematics_george(xd, q_open_d);

handle2 = robot.plot();
