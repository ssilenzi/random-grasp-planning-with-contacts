function [exit, nodes_out, edges_out] = ...
    implement_wanted_release_real_robot(node_s, environment, ...
    edge_types, edge_weights, target, n_nodes, dt_max)

% IMPLEMENT WANTED RELEASE

% Some params
verbose = true;

% Getting the start node properties
[~, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, Cont_h_s, Cp_h_s, Cn_h_s, ...
    dir_s, dist_s, ~] = get_node_properties(node_s);

% Copying already some outputs for positioning (some will change, others won't)
[ID_f, box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
    Cont_h_f, Cp_h_f, Cn_h_f, dir_f, dist_f, prev_pos_f] = ...
    copy_node_properties(n_nodes+1, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
    Cont_h_s, Cp_h_s, Cn_h_s, dir_s, dist_s, true);

robot_f.plot([], false, gca);

% Assigning at first empty nodes_out and edges_out
nodes_out = [];
edges_out = [];

if isempty(Cp_h_s) || isempty(Cn_h_s)
    disp('REL - No hand contacts! Cannot release from this config!');
    exit = 0;
    return;
end

% Get starting object position as row
Co_s = box_s.T(1:3,4).';

% Trying to get the wanted release

found = false;
while ~found
    % Moving the hand in a release pose
    robot_tmp = copy(robot_f);
    robot_tmp.get_release_config(Cp_h_f, Cn_h_f, Co_s, box_s);
    
    % Checking for joint limits or collisions
    is_viol_gripper_joints = ...
        any(robot_tmp.q < robot_tmp.lo_joint_lims) || ...
        any(robot_tmp.q > robot_tmp.up_joint_lims);
    [~, self_coll_pair_id, world_coll_pair_id] = ...
        robot_tmp.build_check_collisions(box_f, environment);
    
    % If problems, bad!
    if is_viol_gripper_joints || ~isempty(self_coll_pair_id) || ~isempty(world_coll_pair_id)
        if verbose
            if is_viol_gripper_joints
                disp('REL - Joint limits detected while moving hand IK');
            end
            if ~isempty(self_coll_pair_id)
                disp('REL - Collision robot detected while moving hand IK');
            end
            if ~isempty(world_coll_pair_id)
                disp('REL - Collision env detected while moving hand IK');
            end
        end
        error('Cannot release!!!');
    else
        if verbose
            disp('REL - Found a good hand pose while releasing');
        end
        robot_f = robot_tmp;
        robot_f.plot([], false, gca);
        % IF HERE, FOUND GOOD RELEASE
        found = true;
    end
    
end

if ~found
    exit = 0;
    return;
end

% Assigning the remanining stuff
Cone_f = Cone_s;
Cont_h_f = false;
Cp_h_f = [];
Cn_h_f = [];
prev_pos_f = false;

% Creating the moved node
node_rel = create_node(ID_f, box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
    Cont_h_f, Cp_h_f, Cn_h_f, dir_f, dist_f, prev_pos_f);

% The output node
nodes_out = node_rel;

% Creating weighted edge between nodes
e_type_sf1 = edge_types{3};
e_weight_sf1 = edge_weights(3);
edge_rel = table({e_type_sf1}', e_weight_sf1, ...
    'VariableNames',{'Type','Weight'});

% Two output edges
edges_out = edge_rel;

% All is well now
exit = 1;

end

