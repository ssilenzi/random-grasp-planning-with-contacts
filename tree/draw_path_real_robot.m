function fig_h = draw_path_real_robot(env,box_ini,box_fin,robot,G,P_rand,...
    ax_range,az,el)

% DRAW PATH - Draw the environment, and then draw the stuff of all the
% provided path (sequence of nodes)

video = true;

if video
    v = VideoWriter('rand_path.avi');
    v.Quality = 100;
    v.FrameRate = 1; % How many frames per second.
    open(v);
end

% Creating the first figure with robot started
fig_h = figure('Color',[1 1 1], 'pos',[0 0 800 800], ...
    'WindowState', 'maximized');
rob_h = robot.plot([], true, gca);
tot_h = plot_scenario(env, box_ini, box_fin, ax_range, az, el);

disp('Length of path is '); disp(length(P_rand));

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
    % Cone_s = node_s.Cone{1};
    Cont_h_s = node_s.Cont_h; % not a cell as only true or false
    Cp_h_s = node_s.Cp_h{1};
    Cn_h_s = node_s.Cn_h{1};
    
	rob_h = robot_s.plot([], false, gca);
    
    plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 0.5 0.5], true);
    plot_contacts(Cp_h_s, Cn_h_s, [1 0 1], 0.5e-1);
%     plot_contacts(Cp_e_s, Cn_e_s, [1 0 1]);
    
    view(az, el);
    
    pause(0.5);
%         pause;
    
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

