%% TEST EXAMPLE CONTACT-BASED GRASP RRT %%
% Use this for paper videos and images

clear all; clc;

%% Define main parameters

% Saved experiment files
% file_name = 'Tree_book_on_shelf_good_fin.mat';
file_name = 'book_on_table_cluttered1.mat';

% Load the file
% load(fullfile('mat_tree/final/', file_name));
load(fullfile('mat_tree/old/', file_name)); % G_final = G_out;

% Define plot constants
% shelf
% axis_range = [-5 5 0 10 0 10];
% azim = 50; % 50; 
% elev = 30;
% cluttered
% axis_range = [-5 5 0 10 0 10];
% azim = 50; % 50; 
% elev = 30;

%% Preliminary plots
% Get and draw random long paths
fin_ID = height(G_final.Nodes);
% fin_ID = randsample(2:height(G_final.Nodes),1);
P_rand = shortestpath(G_final,1,fin_ID);
figure_hand2 = draw_path(env,obj_fin,G_final,P_rand,...
    axis_range,azim,elev);

% To draw a single node
% ID_rand = 1;
% figure_hand3 = draw_node(env,obj_fin,G_final,ID_rand,...
%     axis_range,azim,elev);

%% Adjusting nodes
% box_tmp = G_final.Nodes(P_rand(3),:).Object{1};
% [Cp_e_tmp, Cn_e_tmp] = get_contacts(environment, box_tmp, box_tmp.T);
% Cone_tmp = pfc_analysis(Cp_e_tmp, Cn_e_tmp, 3);
% d_pose = Cone_tmp(:,2)*1;
% 
% for i = 3:5
%     
%     node_s = G_final.Nodes(P_rand(i),:);
%     node_out = move_stuff_in_node(node_s, d_pose, environment);
%     
%     pause;
%     G_final.Nodes(P_rand(i),:) = node_out;
%     
% end

%% Saving the tree
% save('Tree_tempX.mat','G_final','env','obj_fin','axis_range','azim','elev')
