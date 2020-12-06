function [exit,box_f,robot_f,Cp_e_f,Cn_e_f,Cone_f,Cont_h_f,Cp_h_f,Cn_h_f] = ...
    implement_positioning(box_s,robot_s,Cp_e_s,Cn_e_s,Cone_s, ...
    Cont_h_s,Cp_h_s,Cn_h_s,environment)
% IMPLEMENT POSITIONING - From the input node, position the hand on the
% object. If hand contacts are already present (after spawning), just
% position; otherwise, choose random contacts and position

%   Inputs:     properties of the starting node
%   Outputs:    properties of the finishing node and environment (list)
%               exit is true if a node was found, else it is false

% Some params
n_try = 3;
plot_hand_conts = false;
plot_rob = false;
verbose = false;
num_contacts_hand = 2;  % TODO Get this from outside

% Assigning already some outputs (some will change, others won't)
box_f = box_s;
robot_f = copy(robot_s);
Cp_e_f = Cp_e_s;
Cn_e_f = Cn_e_s;
Cone_f = Cone_s;
Cont_h_f = Cont_h_s; % this one is false but will become true
Cp_h_f = Cp_h_s;
Cn_h_f = Cn_h_s;

% Spawning the robot in a non colliding pose after choosing random contacts
found = false;
for i = 1:n_try
    
    % Getting random contacts on free faces only if no hand contacts yet
    [Cp_h_f, Cn_h_f] = get_random_contacts_on_box_partial(box_s,  ...
        num_contacts_hand, Cp_e_s, Cn_e_s, plot_hand_conts);
    
    % Check if there are coincident contacts, for avoiding G loosing rank
    for j = 1:size(Cp_h_f,1)
        for k = j+1:size(Cp_h_f,1)
            if (norm(Cp_h_f(j,:) - Cp_h_f(k,:)) < 0.01)
                if true
                    warning('POS - Near contacts detected');
                    Cp_h_f(j,:)
                    Cp_h_f(k,:)
                end
                continue; % Two similar contact positions, try others
            end
        end
    end
    
    % Loading the hand in a starting pose
    q0 = robot_f.get_starting_config_george(Cp_h_f, Cn_h_f);
    robot_f.set_config(q0);

	% Moving robot to contacts
    [robot_f, success] = move_robot_to_points(robot_f,Cp_h_f);
    
    % Checking rob env collisions
    if ~success || robot_f.check_collisions({box_s}) || ...
            robot_f.check_collisions(environment)
        if verbose
            disp('POS - Collision hand env detected');
        end
        % go further with the next random points
    else
        if verbose
            disp('POS - Found a good hand pose');
        end
        if plot_rob
           rob_handle0 = robot_f.plot(); % TODO: maybe return this handle 
        end
        found = true;
        Cont_h_f = true; % THERE IS OBJ-HAND CONTACT
        break; % the first ntry that is ok exit for
    end
    
end

if ~found
    exit = false;
    return;
end

exit = true;

end

