function [G_out, ind_sol, nearest] = ...
    create_franka_graph(G, env, obj_fin, n_expand, tol,...
    edge_types, edge_weights);

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
    franka_m = copy(franka_s);
    franka_m.set_config(q_arr);
    franka_m.plot([], false, gca);
    plot3(Cp(:,1), Cp(:,2), Cp(:,3), 'r*')
    
    % Projecting the contacts on the nearest faces of box
    box_m = copy(box_s);
    
    
end

end

