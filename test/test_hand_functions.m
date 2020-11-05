%% For testing hand functions for starting and ik

close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Load the environment and the box (both initial and final poses)
run('book_on_shelf.m')
axis([-5 5 0 15 0 15]); % Change the axis and view
view(50, 30);
legend off;

% Get contacts with the environment and plot
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

robot.compute_differential_inverse_kinematics_george(xd);

handle2 = robot.plot();

