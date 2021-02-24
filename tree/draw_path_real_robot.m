function fig_h = draw_path_real_robot(env,box_ini,box_fin,robot,G,P_rand,...
    ax_range,az,el)

% DRAW PATH - Draw the environment, and then draw the stuff of all the
% provided path (sequence of nodes)

video = true;

if video
    v = VideoWriter('rand_path.avi');
    v.Quality = 100;
    v.FrameRate = 2; % How many frames per second.
    open(v);
end

% Getting first node robot
node_1 = G.Nodes(1,:); % row corresponding to 1 (for shelf 583)
robot_s = node_1.Robot{1};
% robot_s = robot;
box_s = box_ini;

% Ad hoc
% For sliding
% node_s2 = G.Nodes(P_rand(end-1),:); % row corresponding to r_nodeID_s
% robot_s2 = node_s2.Robot{1};
% robot_s.q(1) = robot_s.q(1) - 0.2;
% robot_s.q(2) = robot_s.q(2) - 0.4;
% robot_s.q(4) = robot_s.q(4) - 0.2;
% robot_s.q(7) = robot_s2.q(7) - 0.5;
% For cluttered
node_s2 = G.Nodes(P_rand(end-1),:); % row corresponding to r_nodeID_s
robot_s2 = node_s2.Robot{1};
robot_s.q(1) = robot_s.q(1) - 0.7;
robot_s.q(2) = robot_s.q(2) - 0.4;
robot_s.q(4) = robot_s.q(4) - 0.4;
robot_s.q(5) = robot_s.q(5) + 0.8;
robot_s.q(7) = robot_s2.q(7) - 0.9;

% Creating the first figure with robot started
fig_h = figure('Color',[1 1 1], 'pos',[0 0 800 800], ...
    'WindowState', 'maximized');
% rob_h = robot.plot([], false, gca);
rob_h = robot_s.plot([], false, gca);
cont_h = {};
tot_h = plot_scenario(env, box_ini, box_fin, ax_range, az, el);
box_h = plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 0 1], true);
axis(ax_range);
axis off; grid off;
view(az, el);

pause(0.5);

if video
    frame = getframe(gcf);
    writeVideo(v,frame);
end

disp('Length of path is '); disp(length(P_rand));

for i = 2:length(P_rand)
    
    disp('Iteration '); disp(i);
%     disp('Drawing node '); disp(P_rand(i));
    
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
    
%     rob_h = robot.plot([], false, gca);
	rob_h = robot_s.plot([], false, gca);
    
    delete(box_h{1});
    delete(box_h{2});
    if i ~= 2
        delete(cont_h{1});
        delete(cont_h{2});
    end
    box_h = plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 0 1], true);
    if ~ isempty(Cp_h_s)
        cont_h = plot_contacts(Cp_h_s, Cn_h_s, [1 0 1], 0.5e-1);
    end
%     plot_contacts(Cp_e_s, Cn_e_s, [1 0 1]);
    
    axis(ax_range);
    axis off; grid off;
    view(az, el);
    
    pause(0.5);
%         pause;
    
    if video
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    
end

% Doing a final release
% For shelf
% node_s = G.Nodes(P_rand(end),:); % row corresponding to r_nodeID_s
% robot_s = node_s.Robot{1};
% robot_s.q(2) = robot_s.q(2) + 1; % Changing the robot config to release
% robot_s.q(8:9) = [0.04; 0.04];
% For sliding
% node_s = G.Nodes(1,:); % row corresponding to r_nodeID_s
% robot_s = node_s.Robot{1};
% node_s2 = G.Nodes(P_rand(end-1),:); % row corresponding to r_nodeID_s
% robot_s2 = node_s2.Robot{1};
% robot_s.q(1) = robot_s.q(1) - 0.2;
% robot_s.q(2) = robot_s.q(2) - 0.4;
% robot_s.q(4) = robot_s.q(4) - 0.25;
% robot_s.q(5) = robot_s.q(5) + 0.1;
% robot_s.q(7) = robot_s2.q(7) - 0.5;
% For cluttered
node_s = G.Nodes(P_rand(end),:); % row corresponding to r_nodeID_s
robot_s = node_s.Robot{1};
robot_s.q(1) = robot_s.q(1) - 0.2;
robot_s.q(2) = robot_s.q(2) - 1;
robot_s.q(3) = robot_s.q(3) + 0.7;
robot_s.q(4) = robot_s.q(4) - 0.1;
robot_s.q(5) = robot_s.q(5) + 0.8;
robot_s.q(6) = robot_s.q(6) - 0.7;
robot_s.q(7) = robot_s.q(7) - 0.4;
robot_s.q(8:9) = [0.04; 0.04];
% Show releasing robot
delete(cont_h{1});
delete(cont_h{2});
rob_h = robot_s.plot([], false, gca);


pause(0.5);

if video
    frame = getframe(gcf);
    writeVideo(v,frame);
end

pause(0.5);

if video
    frame = getframe(gcf);
   	writeVideo(v,frame);
    close(v);
end

end

