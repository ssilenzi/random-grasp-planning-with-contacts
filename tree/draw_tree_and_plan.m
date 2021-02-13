function fig_h = draw_tree_and_plan(env,box_ini,box_fin,robot,G,P_rand,...
    ax_range,az,el)

% DRAW TREE - Draw the environment, and then draw the stuff of all the
% nodes of the input tree

draw_hand = false;
skip = 10;
video = true;

if video
    v = VideoWriter('rand_path.avi');
    v.Quality = 100;
    v.FrameRate = 2; % How many frames per second.
    open(v);
end

fig_h = figure('Color',[1 1 1], 'pos',[0 0 800 800], ...
    'WindowState', 'maximized');
plot_environment(env, true);
plot_box(box_fin.l, box_fin.w, box_fin.h, ...
    box_fin.T, [0 0 0], true)
tot_h = plot_scenario(env, box_ini, box_fin, ax_range, az, el);
axis(ax_range);
axis off; grid off;
view(az, el);
legend off;

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
    Cont_h_s = node_s.Cont_h; % not a cell as only true or false
    Cp_h_s = node_s.Cp_h{1};
    Cn_h_s = node_s.Cn_h{1};
    
%     plot_contacts(Cp_e_s, Cn_e_s, [1 0 1]);
    plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 1 0], false, 0.1, 0.3);
%     plot_contacts(Cp_h_s, Cn_h_s, [1 0 1]);
    
    if video
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
end

for i = 2:length(P_rand)
    
    disp('Iteration '); disp(i);
    disp('Drawing node '); disp(P_rand(i));
    
    % Getting the node and properties
    node_s = G.Nodes(P_rand(i),:); % row corresponding to r_nodeID_s
    
    % Get the properties of the start node (not the global start)
    box_s = node_s.Object{1};
    robot_s = node_s.Robot{1};
    Cp_e_s = node_s.Cp_e{1};
    Cn_e_s = node_s.Cn_e{1};
    Cp_h_s = node_s.Cp_h{1};
    Cn_h_s = node_s.Cn_h{1};
    
    box_h = plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 0 1], true);
%     plot_contacts(Cp_h_s, Cn_h_s, [1 0 1], 0.5e-1);
%     plot_contacts(Cp_e_s, Cn_e_s, [1 0 1]);
    
    axis(ax_range);
    axis off; grid off;
    view(az, el);
    
    if video
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    
end

if video
    frame = getframe(gcf);
   	writeVideo(v,frame);
    close(v);
end

end

