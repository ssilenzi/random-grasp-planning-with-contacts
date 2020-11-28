% Get a starting config for the hand, avoiding collision with the object
% and the environment

close all
clear
clc
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Load the environment and the box (both initial and final poses)
run('book_on_shelf_no_target.m')
% run('book_on_shelf_no_other_books.m')
% run('book_on_table.m')
% run('book_on_table_cluttered_no_target.m')
axis([-2, 6, -5, 15, 0, 15]) % Change the axis and view
axis equal
view([1, 1, 1])
zoom(1)
legend off

% Get contacts with the environment and plot
[Cp, Cn] = get_contacts(environment, box_object, box_object.T);
% plot_contacts(Cp, Cn);

% Get accessible faces
i_faces = get_free_box_faces(box_object, Cp, Cn);
plot_box_face(box_object, i_faces);

% Sample some random point on box free faces and try to close the hand on
% them. Using closed loop inverse kinematics and stack of tasks. If the
% hand collides the object, discard the trial and choose other random
% points on faces.
grippers = 10;
try_max = 10;
plot_initial_conf = false;
robot = load_gripper('hand_example');

handle = {};
for ngripper = 1:grippers
    for ntry = 1:try_max
        % Get random points on object faces
        p = get_random_points_on_box_faces(box_object, i_faces, 2);
        n = zeros(size(p));
        for i = 1:size(p,1)
            i_face = get_faces_from_points_indexes(box_object, p(i,:));
            n(i,:) = box_object.face_normals(i_face,:);
        end

        % Transform random points to global reference system and plot
        p_global = transform_points(p, box_object.T);
        n_global = transform_vectors(n, box_object.T);
        handle{i,1} = plot_contacts(p_global, n_global, [1 0 1]);

        % Get the hand in the starting config
        q = robot.get_starting_config(p_global, n_global);
        robot.set_config(q);
        if plot_initial_conf
            handle{i,1} = [handle{i,1}, robot.plot()];
        end

        % Getting the current wrist pose
        x_now = robot.get_forward_kinematics();
        x_wrist = x_now(:,:,3);

        % Setting the desired points and IK
        xd(:,:,1) = [eye(3) p_global(1,1:3).'; [0 0 0 1]];
        xd(:,:,2) = [eye(3) p_global(2,1:3).'; [0 0 0 1]];
        xd(:,:,3) = x_wrist;
        q_open_d = robot.q(7:8);
        success = robot.differential_inverse_kinematics(xd, q_open_d);
        if ~success || robot.check_collisions(all_boxes)
            delete(handle{i,1}) % clean the chosen starting config
            % go further with the next random points
        else
            handle{i,2} = robot.plot();
            break % the first ntry that is ok, is the way to go
        end
    end
    fprintf('Gripper %2d number of trials %d\n', ngripper, ntry)
end

saveas(gcf, fullfile('..', 'figures', 'hand_collisions.png'))