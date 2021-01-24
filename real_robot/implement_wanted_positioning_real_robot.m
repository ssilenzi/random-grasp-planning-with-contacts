function [exit, nodes_out, edges_out] = ...
            implement_wanted_positioning_real_robot(node_s, environment, ...
            edge_types, edge_weights, target, n_nodes, dt_max);
        
% IMPLEMENT WANTED POS

% Some params
verbose = true;
num_contacts_hand = 2;  % TODO Get these two from outside

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

% Get starting object position as row
Co_s = box_s.T(1:3,4).';

% Show  accessible faces
[i_faces, i_partial] = get_free_box_faces_partial(box_s, Cp_e_s, Cn_e_s);
disp('POS - The acc. faces are '); disp(i_faces);
disp('POS - The partial acc. faces are '); disp(i_partial);
h_filled = plot_box_face(box_s, i_faces);

% Getting the wanted faces
good = false;
chosen_faces = [];
id = [];
while ~good
    disp('POS - The chosen faces are '); disp(chosen_faces);
    prompt = 'POS - Give me another face id ';
    id = input(prompt);
    if any(ismember(id,i_faces))
        chosen_faces = [chosen_faces, id];
        if length(chosen_faces) >= 2
            good = true;
            disp('POS - The final chosen faces are '); disp(chosen_faces);
        end
    else
        disp('POS - The chosen id is not good give me another. ');
    end
end

% Are the chosen faces symmetric?
good = false;
sym = [];
sym_bool = [];
while ~good
    prompt = 'POS - Are the chosen faces opposite? [1] yes, [0] no. ';
    sym = input(prompt);
    if (sym ~= 0 && sym ~= 1)
        disp('POS - The chosen id is not good give me another. ');
    else
        if sym == 1
            disp('POS - Opposite faces, Symmetric contacts!');
            sym_bool = true;
        else
            disp('POS - Non opposite faces!');
            sym_bool = false;
        end
        good = true;
    end
end

% Trying to get the wanted positioning
found = false;
while ~found
    
    % Getting random contacts on the given faces until satisfied
    good = false;
    while ~good
        
        [Cp_h_f, Cn_h_f] = get_random_contacts_on_given_faces(box_s,  ...
            num_contacts_hand, Cp_e_s, Cn_e_s, chosen_faces, i_partial, sym_bool);
        
        % Checking if distant contacts (not possible for gripper)
        dist_conts = false;
        for j = 1:size(Cp_h_f,1)
            for k = j+1:size(Cp_h_f,1)
                if (norm(Cp_h_f(j,:) - Cp_h_f(k,:)) > 0.08)
                    if verbose
                        disp('POS - Distant contacts for franka!');
                    end
                    dist_conts = true;
                end
            end
        end        
        if dist_conts
            break; % Two distant contact positions, try others
        end
        
        h_conts = plot_contacts(Cp_h_f, Cn_h_f, [1 0 1], 0.1);
        
        % Asking if contacts acceptable
        prompt = 'POS - Are the contacts good? [1] yes, [0] no. ';
        good_conts = input(prompt);
        if (good_conts ~= 0 && good_conts ~= 1)
            disp('POS - The chosen id is not good give me another. ');
        else
            if good_conts == 1
                disp('POS - Good contacts, going on!');
                good = true;
            else
                disp('POS - Bad contacts, get others!');
                delete(h_conts{1});
                delete(h_conts{2});
            end
        end
    end
    
    if dist_conts
        continue; % Two distant contact positions, try others
    end
    
    % Loading the hand in a starting pose
    robot_tmp = copy(robot_f);
    robot_tmp.get_starting_config(Cp_h_f, Cn_h_f, Co_s, box_s);
    robot_tmp.plot([], false, gca);

	% Moving robot to contacts
    [robot_tmp, ~] = move_franka_to_points(robot_tmp,Cp_h_f);
    
    % Checking for joint limits or collisions
    is_viol_gripper_joints = ...
        any(robot_tmp.q < robot_tmp.lo_joint_lims) || ...
        any(robot_tmp.q > robot_tmp.up_joint_lims);
    [~, self_coll_pair_id, world_coll_pair_id] = ...
        robot_tmp.build_check_collisions(box_f, environment);
    
    % If problems, continue and choose other points else go on
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
        % choose other random points
        delete(h_conts{1});
      	delete(h_conts{2});
        continue;
    else % Go on!
        if verbose
            disp('POS - Found a good hand pose while positioning');
        end
        robot_f = robot_tmp;
        robot_f.plot([], false, gca);
        Cont_h_f = true; % THERE IS OBJ-HAND CONTACT
        % IF HERE, FOUND GOOD POSITIONING
        found = true;
        break;
    end
    
end

if ~found
    warning('POS - Could not find a good positioning! Exiting!');
    exit = 0;
    return;
end

% Creating the two positioned and moved nodes
node_pos = create_node(ID_f, box_f, robot_f, Cp_e_f, Cn_e_f, Cone_f, ...
    Cont_h_f, Cp_h_f, Cn_h_f, dir_f, dist_f, prev_pos_f);

% Two output nodes
nodes_out = node_pos;

% Creating weighted edges between nodes
e_type_sf = edge_types{1};
e_weight_sf = edge_weights(1);
edge_pos = table({e_type_sf}', e_weight_sf, ...
    'VariableNames',{'Type','Weight'});

% Two output edges
edges_out = edge_pos;

% All is well now
exit = 1;

end

