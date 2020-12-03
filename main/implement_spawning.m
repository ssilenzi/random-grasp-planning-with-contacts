function [exit,box_f,robot_f,Cp_e_f,Cn_e_f,Cone_f,Cont_h_f,Cp_h_f,Cn_h_f] = ...
    implement_spawning(box_s,robot_s,Cp_e_s,Cn_e_s,Cone_s, ...
    Cont_h_s,Cp_h_s,Cn_h_s,environment)

% NOT USED IN THE GRAPH AS OF NOW

% IMPLEMENT SPAWNING - From the input node, spawns the robot after choosing
% random contacts. The hand is not positioned yet nor the chosen contacts
% will be saved

% Attention: This transition is implemented from the starting node or after
% a release so as to go towards a new good position for contacts.
% The spawning edge is an addition for aiding a good following positioning.
% Therefore, there we do also a positioning test

%   Inputs:     properties of the starting node
%   Outputs:    properties of the finishing node and environment (list)
%               exit is true if a node was found, else it is false

% Cp_h_s, Cn_h_s are empty and will stay empty in Cp_h_f, Cn_h_f

% Some params
n_try = 10;
plot_hand_conts = false;
plot_rob = true;
verbose = true;

% Assigning already some outputs (some will change, others won't)
box_f = box_s;
robot_f = copy(robot_s);
Cp_e_f = Cp_e_s;
Cn_e_f = Cn_e_s;
Cone_f = Cone_s;
Cont_h_f = Cont_h_s;    % this one is false
Cp_h_f = Cp_h_s;        % this is empty and will stay empty
Cn_h_f = Cn_h_s;        % this is empty and will stay empty

% Spawning the robot in a non colliding pose after choosing random contacts
found = false;
robot_tmp = [];
for i = 1:n_try
    
    % Getting random contacts on free faces
    [Cp_h_f, Cn_h_f] = get_random_contacts_on_box_partial(box_s,  ...
        robot_s.n_dof, Cp_e_s, Cn_e_s, plot_hand_conts);
    
    % Loading the hand in a starting pose
    q0 = robot_f.get_starting_config_george(Cp_h_f, Cn_h_f);
    robot_f.set_config(q0);
    
    % Doing also a positioning test
    robot_tmp = copy(robot_f);
    [robot_tmp, success] = move_robot_to_points(robot_tmp,Cp_h_f);
    
    % Checking rob env collisions at spawn and after
    not_good = ~success || robot_f.check_collisions({box_s}) || ...
        robot_f.check_collisions(environment) || ...
        robot_tmp.check_collisions({box_s}) || ...
        robot_tmp.check_collisions(environment);
    if not_good
        if verbose
            disp('SPAWN - Collision hand env before or after detected');
        end
        % go further with the next random points
    else
        if verbose
            disp('SPAWN - Found a good hand pose');
        end
        if plot_rob
           rob_handle0 = robot_f.plot(); % TODO: maybe return this handle 
        end
        found = true;
        break; % the first ntry that is ok exit for
    end
    
end

if ~found
    exit = false;
    return;
end

exit = true;

end

