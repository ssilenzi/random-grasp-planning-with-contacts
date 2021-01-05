function [exit,nodes_out,edges_out] = ...
    implement_direct_twist2(node_s,environment, ...
    edge_types,edge_weights,target,n_nodes)

%   Inputs:     starting node
%               environment (list)
%               info on edges
%               present number of nodes of the graph
%               the target object pose
%   Outputs:    finishing nodes
%               related edges
%               exit is 2 if direct solution was found, else it is 0

% Some params
verbose = false;
direct_lin_vel = false;
coll_points = 10;
twist_step = 1;      % for direct twist
dt = 0.1;
spaced_vec = 0.1:dt:1;

% Getting the start node properties
[~, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, Cont_h_s, Cp_h_s, Cn_h_s, ...
    dir_s, dist_s, ~] = get_node_properties(node_s);

% Getting the twist needed to go from present pose to target pose
if direct_lin_vel
    [hom_seq, ~, ~, ~] = get_slerp_interpolation(target,box_s, spaced_vec);
    % Getting twist just for getting direction (for future dot product)
    % This is not 100% correct
    twist_d = get_direct_twist(target, box_s, twist_step); 
else
    twist_d = get_direct_twist(target, box_s, twist_step);
end

% Copying to a temporary node
[ID_prev, box_prev, robot_prev, Cp_e_prev, Cn_e_prev, Cone_prev, ...
    Cont_h_prev, Cp_h_prev, Cn_h_prev, dir_prev, dist_prev, prev_pos_prev] = ...
    copy_node_properties(n_nodes, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
    Cont_h_s, Cp_h_s, Cn_h_s, dir_s, dist_s, false);

% Assigning at first empty nodes_out and edges_out
nodes_out = [];
edges_out = [];

% Spawning the robot in a non colliding pose after choosing random contacts
found = false;
for i = 1:length(spaced_vec)
    
    if verbose
        disp('DIRTWIST - iteration no. '); disp(i);
    end
    
    % Copying to previous to a next temporary node
    [ID_next, box_next, robot_next, Cp_e_next, Cn_e_next, Cone_next, ...
        Cont_h_next, Cp_h_next, Cn_h_next, dir_next, dist_next, ...
        prev_pos_next] = copy_node_properties(ID_prev, box_prev, ...
        robot_prev, Cp_e_prev, Cn_e_prev, Cone_prev, Cont_h_prev, ...
        Cp_h_prev, Cn_h_prev, dir_prev, dist_prev, prev_pos_prev);
    
    ID_next = ID_next + 1;
    
    % Changing only the needed stuff: box pose and robot are rotated
    % Rotating object
    if direct_lin_vel
        box_next.T = hom_seq(:,:,i)*box_next.T;
    else
        box_next = twist_moves_object(box_next, twist_d*dt);
    end
        
    % Checking the object env collision
    [bool, coll_type] = check_collisions_box(box_next, environment);
    if bool == true
        if verbose
            fprintf('DIRTWIST - box %s collision\n', coll_type)
        end
        break;
    else
        if verbose
            disp('DIRTWIST - box no collision\n');
        end
    end
    
    % Rotating the robot
    if direct_lin_vel
        Hom_twist_d = hom_seq(:,:,i); % hom. trans. for linear
    else
        Hom_twist_d = twistexp(twist_d*dt); % hom. trans. from twist
    end
   	Cp_h_next = transform_points(Cp_h_prev, Hom_twist_d);      % transforming points
    Cn_h_next = transform_vectors(Cn_h_prev, Hom_twist_d);     % transforming normals
    
    % Wanted wrist transform
    wrist_prev = robot_prev.get_forward_kinematics();
    wrist_prev = wrist_prev(1:4,4,3); % previous wrist position
    wrist_next = Hom_twist_d * wrist_prev;
    wrist_next = wrist_next(1:3);
    
    % Moving robot to contacts
    [robot_next, success] = ...
        move_robot_to_points_and_wrist(robot_next,Cp_h_next,wrist_next);
    
    % Checking rob env collisions
    if ~success || robot_next.check_collisions({box_next},coll_points) ...
            || robot_next.check_collisions(environment,coll_points)
        if verbose
            disp('DIRTWIST - Collision hand env detected while moving hand');
        end
        break;
    else
        if verbose
            disp('DIRTWIST - Found a good hand pose after moving');
        end
    end
    
    % Changing other properties
    dir_next = twist_d;
    dist_next = hom_dist(box_next.T, target.T);
    prev_pos_next = false;
    
    % Creating the moved node
    node_next = create_node(ID_next, box_next, robot_next, Cp_e_next, ...
        Cn_e_next, Cone_next, Cont_h_next, Cp_h_next, Cn_h_next, ...
        dir_next, dist_next, prev_pos_next);
    
    % Saving the moved node
    nodes_out = [nodes_out; node_next];
    
    % Creating weighted edge between nodes
    e_type_np = edge_types{3};
    e_weight_np = edge_weights(3);
    edge_np = table({e_type_np}', e_weight_np, ...
        'VariableNames',{'Type','Weight'});
    
    % Two output edges
    edges_out = [edges_out; edge_np];
    
    % If last iteration and here, we found direct interpolation
    if i == length(spaced_vec)
        found = true;
    end
    
    % Copying to prev node the next node for next iteration
    [ID_prev, box_prev, robot_prev, Cp_e_prev, Cn_e_prev, Cone_prev, ...
        Cont_h_prev, Cp_h_prev, Cn_h_prev, dir_prev, dist_prev, ...
        prev_pos_prev] = copy_node_properties(ID_next, box_next, ...
        robot_next, Cp_e_next, Cn_e_next, Cone_next, Cont_h_next, ...
        Cp_h_next, Cn_h_next, dir_next, dist_next, prev_pos_next);
    
end

if ~found
    exit = 0;
    if verbose
        disp('DIRTWIST - NOT FOUND A GOOD DIRECT SOLUTION!');
    end
    return;
end

% All is well now
exit = 2;

end

