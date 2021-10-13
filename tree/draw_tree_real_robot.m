function fig_h = draw_tree_real_robot(env,box_fin,G,ax_range,az,el)

% DRAW TREE - Draw the environment, and then draw the stuff of all the
% nodes of the input tree

draw_robot = false;

% Loading the robot
robot = load_gripper(robot_name);

% Creating the first figure with robot started
fig_h = figure('Color',[1 1 1], 'pos',[0 0 800 800], ...
    'WindowState', 'maximized');
rob_h = robot.plot([], true, gca);
tot_h = plot_scenario(env, box_obj, box_fin, ax_range, az, el);

n_nodes = height(G.Nodes); % no. of rows of table of Nodes
for i = 1:n_nodes
    disp('Drawing node '); disp(i);
    % Getting the node and properties
    node_s = G.Nodes(i,:); % row corresponding to r_nodeID_s
    
    % Get the properties of the start node (not the global start)
    box_s = node_s.Object{1};
    robot_s = node_s.Robot{1};
    Cp_e_s = node_s.Cp_e{1};
    Cn_e_s = node_s.Cn_e{1};
    % Cone_s = node_s.Cone{1};
    Cont_h_s = node_s.Cont_h; % not a cell as only true or false
    Cp_h_s = node_s.Cp_h{1};
    Cn_h_s = node_s.Cn_h{1};
    
    
%     plot_contacts(Cp_e_s, Cn_e_s, [1 0 1]);
    if Cont_h_s && draw_robot
        robot_s.plot([], true, gca);
    end
    plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 0 0], true);
%     plot_contacts(Cp_h_s, Cn_h_s, [1 0 1]);
end

end

