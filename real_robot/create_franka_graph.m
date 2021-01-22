function [G_out, ind_sol, nearest] = ...
    create_franka_graph(G, env, obj_fin, n_expand,...
    edge_types, edge_weights)

rosshutdown
rosinit('172.16.0.6');

% Creating subscribers and tf echoer
joint_states_sub = rossubscriber('/joint_states');
tftree = rostf;

for i = 1:n_expand
    
    pause;
    
    % Getting needed info
    joints_msg = receive(joint_states_sub, 5);
    q_arr = joints_msg.Position;
    Tp1 = getTransform(tftree, 'world', 'panda_leftfinger_tip');
    cp1 = Tp1.Transform.Translation;
    Tp2 = getTransform(tftree, 'world', 'panda_rightfinger_tip');
    cp2 = Tp2.Transform.Translation;
    Cp = [cp1.X, cp1.Y, cp1.Z;
        cp2.X, cp2.Y, cp2.Z];    
    
    % Get last node from the tree
    n_nodes = height(G.Nodes); % no. of rows of table of Nodes
    node_s = G.Nodes(n_nodes,:); % row corresponding to r_nodeID_s
    
    % Getting the last node robot
    [~, box_s, franka_s, ~, ~, ~, ~, ~, ~, ...
        ~, ~, ~] = get_node_properties(node_s);
    
    % Creating the next robot
    franka_f = copy(franka_s);
    franka_f.set_config(q_arr);
    franka_f.plot([], false, gca);
%     plot3(Cp(:,1), Cp(:,2), Cp(:,3), 'r*')
    
    % What is this transition? 1 pos, 2 mov, 3 rel
    prompt = 'What is this transition? ';
    tr = [];
    while true
        tr = input(prompt); 
        if (tr == 1 || tr == 2 || tr == 3)
            break;
        end
    end
    
    % If moving, rotating the box according to the franka wrist rotation
    box_f = box_s;
    if (tr == 3) % moving
        T1 = franka_s.T_all(:,:,9);
        T2 = franka_f.T_all(:,:,9);
        T21 = T2*inv(T1);
        box_f.T = T21*box_f.T;
    end
    
    % Other values
    Cont_h_f = true;
    if (tr == 3) 
        Cont_h_f = false;
    end
    dist_f = hom_dist(box_f.T, obj_fin.T);
    
    % Creating the moved node
    node_f = create_node(n_nodes+1, box_f, franka_f, [], [], [], ...
        Cont_h_f, [], [], [], dist_f, []);
    
    % Creating weighted edge between nodes
    e_type_sf = edge_types{tr};
    e_weight_sf = edge_weights(tr);
    edge_f = table({e_type_sf}', e_weight_sf, ...
        'VariableNames',{'Type','Weight'});
    
    % Adding the newly created nodes and edges
  	G = add_nodes_edges_to_graph(G,node_f,edge_f,n_nodes);
    
end

G_out = G;

end

