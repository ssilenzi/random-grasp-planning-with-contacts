function figure_hand = draw_node(environment,target_position,G,ID_rand,...
    axis_range,azim,elev)

% DRAW NODE - Draw the environment, and then draw the stuff of all the
% provided node

figure_hand = figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
% figure_hand.WindowState = 'maximized';
plot_environment(environment, true);
% plot_box(target_position.l, target_position.w, target_position.h, ...
%     target_position.T, [0 0 0], true)
xlabel('z');
ylabel('x');
zlabel('y');
axis(axis_range);
axis equal;
legend off;

set(gca,'visible','off')

handle_r = [];
disp('Drawing node '); disp(ID_rand);

% Getting the node and properties
node_s = G.Nodes(ID_rand,:); % row corresponding to r_nodeID_s

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
% handle_r = robot_s.plot();
plot_box(box_s.l, box_s. w, box_s.h, box_s.T, [0 0 1], true);
% plot_box(target_position.l, target_position. w, target_position.h, ...
%     target_position.T, [0 0 0], true);
% plot_contacts(Cp_h_s, Cn_h_s, [0 1 0]);

view(azim, elev);


end

