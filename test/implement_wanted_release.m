function [exit, nodes_out, edges_out] = ...
    implement_wanted_release(node_s, environment, ...
    edge_types, edge_weights, target, n_nodes, dt_max)

% IMPLEMENT WANTED RELEASE

% Some params
verbose = true;
coll_points = 10;

% Getting the start node properties
[~, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, Cont_h_s, Cp_h_s, Cn_h_s, ...
    dir_s, dist_s, ~] = get_node_properties(node_s);

% Copying already some outputs for positioning (some will change, others won't)
[ID_f, box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
    Cont_h_f, Cp_h_f, Cn_h_f, dir_f, dist_f, prev_pos_f] = ...
    copy_node_properties(n_nodes+1, box_s, robot_s, Cp_e_s, Cn_e_s, Cone_s, ...
    Cont_h_s, Cp_h_s, Cn_h_s, dir_s, dist_s, true);

rob_handle0 = robot_f.plot();

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

% rob_handle = robot_f.plot();

% Trying to get the wanted release

found = false;
while ~found
    % Moving the hand in a release pose
    robot_tmp = copy(robot_f);
    sig2 = robot_tmp.get_release_config_george(Cp_h_s, Cn_h_s, Co_s);
    robot_tmp.set_act(sig2);
    %     rob_handle = robot_tmp.plot();

	% Checking collisions?
    box_coll = robot_tmp.check_collisions({box_s},coll_points);
    world_coll = robot_tmp.check_collisions(environment,coll_points);
    
    % If problems, continue and choose other points else go on
    if box_coll || world_coll
        if verbose
%             rob_handle = robot_tmp.plot();
            if box_coll
                disp('REL - Box Collision');
            end
            if world_coll
                disp('REL - World Collision');
            end
        end
        disp('REL - Cannot release!!!');
        rob_handle = robot_tmp.plot();
        break;
    else
        if verbose
            disp('REL - Found a good hand pose while releasing');
        end
        robot_f = robot_tmp;
        rob_handle = robot_f.plot();
        % IF HERE, FOUND GOOD RELEASE
        found = true;
    end
    
end

if ~found
    exit = 0;
    return;
end

% Just pausing for the user to confirm the goodness of the robot pose
disp('REL - Please confirm the robot is good');
pause;
delete(rob_handle0);
delete(rob_handle);

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