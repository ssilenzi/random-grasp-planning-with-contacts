function [exit, nodes_out, edges_out] = ...
    implement_wanted_moving_real_robot(node_s, environment, ...
    edge_types, edge_weights, target, n_nodes, dt_max)

% IMPLEMENT WANTED MOVING

% Some params
n_try = 20;
verbose = true;
d_pose_tol = 0.001;

% Getting the start node properties
[~, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, Cont_h_s, Cp_h_s, Cn_h_s, ...
    dir_s, dist_s, ~] = get_node_properties(node_s);

% Copying already some outputs for moving (some will change, others won't)
[ID_f1, box_f1, robot_f1, Cp_e_f1, Cn_e_f1, ~ , ...
    Cont_h_f1, Cp_h_f1, Cn_h_f1, dir_f1, ~, prev_pos_f1] = ...
    copy_node_properties(n_nodes+1, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
    Cont_h_s, Cp_h_s, Cn_h_s, dir_s, dist_s, false);

% Assigning at first empty nodes_out and edges_out
nodes_out = [];
edges_out = [];

% Get starting object position as row
Co_s = box_s.T(1:3,4).';

% Selecting an object motion, and moving until satisfied
found = false;
for i = 1:n_try
    
    % Choosing the vector of cone for motion
    good = false;
    chosen_vecs_inds = [];
    inds = [];
    while ~good
        disp('MOV - The present Cone is '); disp(Cone_s);
        disp('MOV - The chosen vector inds are '); disp(chosen_vecs_inds);
        prompt = 'MOV - Give me another vector ind ';
        inds = input(prompt);
        if inds >= 0 && inds <= size(Cone_s,2)
            chosen_vecs_inds = [chosen_vecs_inds, inds];
            if length(chosen_vecs_inds) >= 2
                good = true;
                disp('MOV - The final chosen vector inds are '); disp(chosen_vecs_inds);
            end
        else
            disp('MOV - The chosen ind is not good give me another. ');
        end
    end
    
    % Creating the combination vector
    alpha = zeros(size(Cone_s,2),1);
    for j = 1:length(chosen_vecs_inds)
        if chosen_vecs_inds(j) == 0
            continue;
        else
            alpha(chosen_vecs_inds(j)) = 1;
        end
    end
        
    % Checking if the new motion is not a "going back"
    if dot(dir_s,Cone_s*alpha) < 0 % choose another combination
        if verbose
            disp('MOV - Continuing, I dont want to go back!');
        end
        continue;
    end
    
    % Moving the object
    [success, box_f1, twist01, d_pose01] = get_pose_from_cone(Cone_s, ...
        box_s, environment, dt_max, alpha);
    if ~success || norm(d_pose01) < d_pose_tol
        if verbose
            disp('MOV - Continuing, could not get a good pose inside Cone');
        end
        continue;
    end
    dir_f1 = twist01; % the new direction of motion
    
    % Check if moved box is good
    h_box = plot_box(box_f1.l, box_f1. w, box_f1.h, box_f1.T, [0 0.5 0.5], true);
    prompt = 'MOV - Do you like the moved box? [1] yes, [0] no. ';
    good = false;
    nice = [];
    while ~good
        nice = input(prompt);
        if (nice ~= 0 && nice ~=1)
            disp('MOV - I dont understand if you like box or not... Tell me again!');
        else
            good = true;
        end
    end
    if nice == 0
        disp('MOV - You dont like new box pose, change it!');
        delete(h_box{1});
        delete(h_box{2});
        continue;
    end
    
    % Finding the moved contact points and normals and new robot config
    Hom_d_pose01 = twistexp(d_pose01); % homogeneous trans. corresponding to obj twist
    Cp_h_f1 = transform_points(Cp_h_s, Hom_d_pose01);      % transforming points
    Cn_h_f1 = transform_vectors(Cn_h_s, Hom_d_pose01);     % transforming normals
    
    h_conts = plot_contacts(Cp_h_f1, Cn_h_f1, [1 0 1], 0.1);
    
    % Wanted wrist transform
    wrist0 = robot_s.get_forward_kinematics();
    wrist0 = wrist0(1:4,4,3); % previous wrist position
    wrist1 = Hom_d_pose01 * wrist0;
    wrist1 = wrist1(1:3);
    
    % Moving robot to contacts
    robot_tmp = copy(robot_f1);
    [robot_tmp, ~] = move_franka_to_points(robot_tmp,Cp_h_f1,wrist1);
    
    % Checking for joint limits or collisions
    is_viol_gripper_joints = ...
        any(robot_tmp.q < robot_tmp.lo_joint_lims) || ...
        any(robot_tmp.q > robot_tmp.up_joint_lims);
    [~, self_coll_pair_id, world_coll_pair_id] = ...
        robot_tmp.build_check_collisions(box_f1, environment);
    
    % Checking rob env collisions
    if is_viol_gripper_joints || ~isempty(self_coll_pair_id) || ~isempty(world_coll_pair_id)
        if verbose
            if is_viol_gripper_joints
                disp('MOV - Joint limits detected while moving hand IK');
            end
            if ~isempty(self_coll_pair_id)
                disp('MOV - Collision robot detected while moving hand IK');
            end
            if ~isempty(world_coll_pair_id)
                disp('MOV - Collision env detected while moving hand IK');
            end
        end
        delete(h_conts{1});
     	delete(h_conts{2});
        delete(h_box{1});
        delete(h_box{2});
        continue;
    else
        if verbose
            disp('MOV - Found a good hand pose after moving');
        end
        robot_f1 = robot_tmp;
        robot_f1.plot([], false, gca);
        Cont_h_f1 = true; % THERE IS OBJ-HAND CONTACT
        % IF HERE, FOUND GOOD MOVING
        found = true;
        break;
    end
    
end

if ~found
    warning('MOV - Could not find a good moving! Exiting!');
    exit = 0;
    return;
end

% Assigning the final cone and checking the distance
Cone_f1 = pfc_analysis(Cp_e_f1, Cn_e_f1, 3);
dist_f1 = hom_dist(box_f1.T, target.T);

% Creating the two positioned and moved nodes
node_mov = create_node(ID_f1, box_f1, robot_f1, Cp_e_f1, Cn_e_f1, Cone_f1, ...
    Cont_h_f1, Cp_h_f1, Cn_h_f1, dir_f1, dist_f1, prev_pos_f1);

% Two output nodes
nodes_out = node_mov;

% Creating weighted edges between nodes
e_type_ff1 = edge_types{2};
e_weight_ff1 = edge_weights(2);
edge_mov = table({e_type_ff1}', e_weight_ff1, ...
    'VariableNames',{'Type','Weight'});

% Two output edges
edges_out =  edge_mov;

% All is well now
exit = 1;

end

