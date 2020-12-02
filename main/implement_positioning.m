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
n_try = 50;
plot_conts = true;
plot_rob = true;

% Assigning already some outputs (some will change, others won't)
box_f = box_s;
robot_f = robot_s;
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
        robot_s.n_dof, Cp_e_s, Cn_e_s, plot_conts);

	% Moving robot to contacts
    [robot_f, success] = move_robot_to_points(robot_f,Cp_h_f);
    
    % Checking rob env collisions
    if ~success || robot_f.check_collisions({box_s}) || ...
            robot_f.check_collisions(environment)
        warning('Collision hand env detected');
        % go further with the next random points
    else
        disp('Found a good hand pose');
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

