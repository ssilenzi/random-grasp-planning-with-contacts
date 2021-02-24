function figure_hand = draw_path(environment,target_position,G,P_rand,...
    axis_range,azim,elev)

% DRAW PATH - Draw the environment, and then draw the stuff of all the
% provided path (sequence of nodes)

video = true;

if video
    v = VideoWriter('rand_path.avi');
    v.Quality = 100;
    v.FrameRate = 2; % How many frames per second.
    open(v);
end

figure_hand = figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
% figure_hand = figure('Color',[1 1 1], 'Position',[0 0 1920 1080]);
figure_hand.WindowState = 'maximized';
plot_environment(environment, true);
plot_box(target_position.l, target_position.w, target_position.h, ...
    target_position.T, [0 0 0], true)
% xlabel('z');
% ylabel('x');
% zlabel('y');
axis(axis_range);
axis equal manual;
axis off manual;
legend off;
view(azim, elev);

% set(gca,'visible','off')

disp('Length of path is '); disp(length(P_rand));

handle_r = [];
handle_b = [];
handle_c = {};
Cp_h_s = [];

for i = 1:length(P_rand)
    
    if i ~= 1
        delete(handle_r);
        delete(handle_b{1});
        delete(handle_b{2});
        if ~isempty(Cp_h_s)
            delete(handle_c{1});
            delete(handle_c{2});
        end
    end
    
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
    
    
    %plot_contacts(Cp_e_s, Cn_e_s, [1 0 1]);
    if i ~= 1
        handle_r = robot_s.plot();
        if ~isempty(Cp_h_s)
            handle_c = plot_contacts(Cp_h_s, Cn_h_s, [1 0 1]);
        end
    end
    handle_b = plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 0 1], true);
%     plot_contacts(Cp_h_s, Cn_h_s, [1 0 1]);
    
    pause(0.5);
%         pause;
    
    if video
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    
end

% Adding a release from last robot pose
% corner
% delete(handle_r);
% delete(handle_c{1});
% delete(handle_c{2});
% Co_s = box_s.T(1:3,4).';
% sig = robot_s.get_release_config_george(Cp_h_s, Cn_h_s, Co_s);
% robot_s.set_act(sig);
% handle_r = robot_s.plot();

pause(0.5);

if video
    frame = getframe(gcf);
   	writeVideo(v,frame);
    close(v);
end

end

