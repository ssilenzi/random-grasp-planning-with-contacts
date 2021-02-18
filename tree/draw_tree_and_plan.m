function figure_hand = draw_tree_and_plan(environment,target_position,G,P_rand,...
    axis_range,azim,elev)

% DRAW TREE - Draw the environment, and then draw the stuff of all the
% nodes of the input tree

draw_hand = false;
video = true;
skip = 1;

if video
    v = VideoWriter('rand_path.avi');
    v.Quality = 100;
    v.FrameRate = 2; % How many frames per second.
    open(v);
end

figure_hand = figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
figure_hand.WindowState = 'maximized';
plot_environment(environment, true);
plot_box(target_position.l, target_position.w, target_position.h, ...
    target_position.T, [0 0 0], true)
xlabel('z');
ylabel('x');
zlabel('y');
axis(axis_range); 
axis equal;
axis off;
view(azim, elev);
legend off;

n_nodes = height(G.Nodes); % no. of rows of table of Nodes
for i = 1:skip:n_nodes
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
    if Cont_h_s && draw_hand
        robot_s.plot();
    end
    plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 1 0], false, 0.1, 0.3);
%     plot_contacts(Cp_h_s, Cn_h_s, [1 0 1]);

    view(azim, elev);
    
%     pause(0.2);
%         pause;
    
    if video
        frame = getframe(gcf);
        writeVideo(v,frame);
    end

end

for i = 1:length(P_rand)
    
    disp('Iteration '); disp(i);
    disp('Drawing plan '); disp(P_rand(i));
    
    % Getting the node and properties
    node_s = G.Nodes(P_rand(i),:); % row corresponding to r_nodeID_s
    
    % Get the properties of the start node (not the global start)
    box_s = node_s.Object{1};
    robot_s = node_s.Robot{1};
    Cp_e_s = node_s.Cp_e{1};
    Cn_e_s = node_s.Cn_e{1};
    % Cone_s = node_s.Cone{1};
    Cont_h_s = node_s.Cont_h; % not a cell as only true or false
    Cp_h_s = node_s.Cp_h{1};
    Cn_h_s = node_s.Cn_h{1};
    
    
    %plot_contacts(Cp_e_s, Cn_e_s, [1 0 1]);
    handle_b = plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 0 1], true);
%     plot_contacts(Cp_h_s, Cn_h_s, [1 0 1]);
    
    view(azim, elev);
    
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

