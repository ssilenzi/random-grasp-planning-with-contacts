function [G_tree] = grasp_object_RRT(environment, object_state, ...
    object_state_final, robot, planner_parameters, ~)
% GRASPOBJECTRRT This function computes a trajectory of states to move an
% object from the "object_state to" the "object_state_final" suposing that 
% it has to be manipulated by the "robot"

%% Variable initialization
% Parameters fot the planner
if ~exist('planner_parameters','var')
    planner_parameters.n_try = 5000;
    planner_parameters.n_check = 100;
    planner_parameters.twist_step = [1 1];
    planner_parameters.try_straight = 0;
    planner_parameters.p_edge = 0;
    planner_parameters.hold_robot_config = false;
end

if planner_parameters.hold_robot_config
    X = robot.get_forward_kinematics();
    object_contacts_positions =  X(1:2,:);
    object_contacts_normals = zeros(size(robot.get_n_contacts(),3));
    for j=1:robot.get_n_contacts()
        i_face = get_faces_from_points_indexes(object_state, ...
            object_contacts_positions(j,:));
        object_contacts_normals(j,:) = object_state.face_normals(i_face,:);
    end
end

G_tree.V = cell(planner_parameters.n_try,1);
G_tree.C = cell(planner_parameters.n_try,1);
G_tree.Cp = cell(planner_parameters.n_try,1);
G_tree.Cn = cell(planner_parameters.n_try,1);
G_tree.done =  false;
G_tree.path_out = []; % this should contain the path
G_tree.cost = []; % this should contain the cost

%% Add initial state to the tree
[Cp, Cn] = get_contacts(environment, object_state, object_state.T);
G_tree.V{1} = object_state;
G_tree.C{1} = pfc_analysis(Cp, Cn, planner_parameters.d);
G_tree.E = [];
G_tree.n_nodes = 1;
G_tree.Cp{1} = Cp;
G_tree.Cn{1} = Cn;

%% RRT algorithm
for i = 1:planner_parameters.n_try
    % This function generates the cone of possible motions
    % Partial force closure analysis
    [t, g_id] =  sample_twist_from_tree(G_tree, object_state_final, ...
        planner_parameters.twist_step, planner_parameters.try_straight, ...
        planner_parameters.p_edge);
    
    if(isempty(t))
        disp('Debugging');
    end
    
    if ~planner_parameters.hold_robot_config
        i_free_faces = get_free_box_faces(object_state, ...
            G_tree.Cp{g_id}, G_tree.Cn{g_id});
        object_contacts_positions = ...
            get_random_points_on_box_faces(object_state, i_free_faces, ...
            robot.get_n_contacts());
        object_contacts_normals = zeros(robot.get_n_contacts(),3);
        for j=1:robot.get_n_contacts()
            i_face = get_faces_from_points_indexes(object_state, ...
                object_contacts_positions(j,:));
            object_contacts_normals(j,:) = ...
                object_state.face_normals(i_face,:);
        end
    end
    Cp_hand = transform_points(object_contacts_positions, object_state.T);
    Cn_hand = transform_vectors(object_contacts_normals, object_state.T);
    
    % robot_state = generate_robot_config(robot, G_tree.Cp{g_id}, ...
    %                   G_tree.Cn{g_id}, Cp_hand, Cn_hand);
    robot_state = generate_robot_config(robot, Cp_hand, Cn_hand);
    % robot.plot();
    object_state_new = twist_moves_object(G_tree.V{g_id}, t);
    
    
    
    if(is_twist_actuatable(t, G_tree.V{g_id},robot_state, Cp, Cn, ...
            Cp_hand, Cn_hand) && ~is_box_in_collision(environment, ...
            object_state_new))
        
        [Cp_new, Cn_new] = get_contacts(environment, object_state_new, ...
            object_state_new.T);
        G_tree.n_nodes = G_tree.n_nodes + 1;
        G_tree.V{G_tree.n_nodes} = object_state_new;
        G_tree.E = [G_tree.E; g_id G_tree.n_nodes];
        
        contact_points_local = transform_points(G_tree.Cp{g_id}, ...
            inv(object_state_new.T));
        normals_local = transform_vectors(G_tree.Cn{g_id}, ...
            inv(object_state_new.T));
        contact_points_new_local = transform_points(Cp_new, ...
            inv(object_state_new.T));
        normals_new_local = transform_vectors(Cn_new, ...
            inv(object_state_new.T));
        if (~(isequal(contact_points_local, contact_points_new_local) ...
                && isequal(normals_local, normals_new_local)))
            G_tree.Cp{G_tree.n_nodes} = Cp_new;
            G_tree.Cn{G_tree.n_nodes} = Cn_new;
            G_tree.C{G_tree.n_nodes} = ...
                pfc_analysis(G_tree.Cp{G_tree.n_nodes}, ...
                G_tree.Cn{G_tree.n_nodes}, planner_parameters.d);
        else
            G_tree.Cp{G_tree.n_nodes} = G_tree.Cp{g_id};
            G_tree.Cn{G_tree.n_nodes} = G_tree.Cn{g_id};
            G_tree.C{G_tree.n_nodes} = G_tree.C{g_id};
        end
        
    end
    
    if (mod(i, planner_parameters.n_check) == 0)
        [res, id] = ...
            is_goal_reachable(environment, G_tree, object_state_final);
        if (res == true)
            G_tree.n_nodes = G_tree.n_nodes +1;
            G_tree.V{G_tree.n_nodes} = object_state_final;
            G_tree.E = [G_tree.E; id G_tree.n_nodes];
            G_tree.done = true;
            G_tree.path_out = extract_path(G_tree);
            return;
        end
    end 
end
end