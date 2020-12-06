% Get a starting config for the hand, avoiding collision with the object
% and the environment

close all
clear
clc
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Load the environment and the box (both initial and final poses)
run('book_vertical_empty.m')
% run('book_on_shelf_no_target.m')
% run('book_on_shelf_no_other_books.m')
% run('book_on_table.m')
% run('book_on_table_cluttered_no_target.m')
axis([-2, 6, -5, 15, 0, 15]) % Change the axis and view
axis equal
view([1, 1, 1])
zoom(1)
legend off

% Get object position as row
Co0 = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp_e0, Cn_e0] = get_contacts(environment, box_object, box_object.T);
% plot_contacts(Cp, Cn);

% Sample some random point on box free faces and try to close the hand on
% them. Using closed loop inverse kinematics and stack of tasks. If the
% hand collides the object, discard the trial and choose other random
% points on faces.
grippers = 5;
try_max = 10;
plot_initial_conf = true;
robot = load_gripper('hand_example', 1.5*ones(4,1));
do_aux_plots = true;
num_hand_conts = 2;

for ngripper = 1:grippers
    for ntry = 1:try_max
        % Get random points on object faces
        [Cp_h0, Cn_h0] = get_random_contacts_on_box_partial(box_object, ...
            num_hand_conts, Cp_e0, Cn_e0, do_aux_plots);

        % Get the hand in the starting config
        q0 = robot.get_starting_config_george(Cp_h0, Cn_h0, Co0);
        robot.set_config(q0);
        if plot_initial_conf
            ht = robot.plot();
        end

        % Moving robot to points
        [robot, success] = move_robot_to_points(robot,Cp_h0);
        
        if ~success || robot.check_collisions(all_boxes, 5)
            warning('Collision hand env detected');
            % go further with the next random points
        else
            disp('Found a good hand pose');
          	ht = robot.plot();
            plot_contacts(Cp_h0, Cn_h0);
            break % the first ntry that is ok, is the way to go
        end
    end
    fprintf('Gripper %2d number of trials %d\n', ngripper, ntry)
end

% saveas(gcf, fullfile('..', 'figures', 'hand_collisions.png'))