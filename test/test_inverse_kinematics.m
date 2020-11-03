% We will start considering that we have a box as object and that the
% environment is known and is composed of boxes only
close all
clear
clc
run(fullfile('..', 'tools', 'resolve_paths.m'))

new_exp = true;

run('book_on_shelf.m')
axis([-5 5 0 10 0 10]);
view(45.7, 50);
[Cp, Cn] = get_contacts(environment, box_object, box_object.T);

if exist(fullfile('..','figures'), 'dir') == false
    mkdir(fullfile('..','figures'))
end
saveas(gcf, fullfile('..', 'figures', '1environment.png'))


figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
plot_contacts(Cp,Cn);
plot_boxes({box_object}, true);
axis([-5 5 0 10 0 10]);
view(45.7, 50);
grid off
legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k')],...
    {'Environment','Initial Position', 'Goal Position'},...
    'Location','northeast');
xlabel('z');
ylabel('x');
zlabel('y');
saveas(gcf, fullfile('..', 'figures', '2box_contact.png'))

view(-43.1, 51.6);
saveas(gcf, fullfile('..', 'figures', 'nbox_contact2.png'))

if ~new_exp
    load('exp_book_on_shelf.mat');
else
    Cone = pfc_analysis(Cp, Cn, 6);
end

dt = 1.5;
for i=1:size(Cone,2)
    figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
    str = sprintf('Twist: [%f %f %f %f %f %f]',Cone(3,i),Cone(1,i),...
        Cone(2,i),Cone(6,i),Cone(4,i),Cone(5,i));
    title(str)
    % plot_contacts(Cp,Cn);
    axis([-5 5 0 10 0 10]);
    view(45.7, 50);
    grid off
    boxn = twist_moves_object(box_object, Cone(:,i)*dt);
    plot_boxes({box_shelf, box_left, box_right, box_wall, boxn}, true);
    fig_name = sprintf('%dtwist_%d.png',i+2, i);
    legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k')],...
        {'Environment','Initial Position', 'Goal Position'},...
        'Location','northeast');
    xlabel('z');
    ylabel('x');
    zlabel('y');
    
    saveas(gcf, fullfile('..', 'figures', fig_name));
    % [Cp, Cn] = get_contacts(environment, boxn, boxn.T);
    % plot_contacts(Cp,Cn);
end

figure('Color',[1 1 1],'Position',[10 10 1000 1000]);
plot_contacts(Cp,Cn);
plot_boxes({box_object}, false)
i_faces = get_free_box_faces(box_object, Cp, Cn);
plot_box_face(box_object, i_faces);

grid off
legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k')], ...
    {'Free Faces','Initial Position', 'Goal Position'}, ...
    'Location','northeast');
axis([-5 5 0 10 0 10]);
view(45.7, 50);
xlabel('z');
ylabel('x');
zlabel('y');
saveas(gcf, fullfile('..', 'figures', '7free_faces.png'))

% sample random points on object faces
if new_exp
    p = get_random_points_on_box_faces(box_object, i_faces, 2);
    n = zeros(size(p));
    for i=1:size(p,1)
        i_face = get_faces_from_points_indexes(box_object, p(i,:));
        n(i,:) = box_object.face_normals(i_face,:);
    end
end

% transformation to global reference system
p_global = transform_points(p, box_object.T);
n_global = transform_vectors(n, box_object.T);
plot_contacts(p_global, n_global, [1 0 1]);

% 3- Get a hand configuration
robot = load_gripper('hand_example');
% place the gripper on the previously randomly sampled points on the object
% faces, computed the joint positions with inverse kinematics.
q = robot.get_starting_config(p_global, n_global);
robot.set_config(q);
legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k'), ...
    plot(NaN,NaN,'-m')],{'Freefaces', 'Initial Position', ...
    'Goal Position', 'Hand Contact Points'}, 'Location','northeast');
axis([-5 5 0 10 0 10]);
view(45.7, 50);

saveas(gcf, fullfile('..', 'figures', '8free_faces_points.png'))

xd(:,:,1) = [eye(3) p_global(1,1:3).'; [0 0 0 1]];
xd(:,:,2) = [eye(3) p_global(2,1:3).'; [0 0 0 1]];
xd(:,:,3) = [eye(3) [0 0 0].';[0 0 0 1]];

robot.compute_differential_inverse_kinematics(xd);
robot.plot();
legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k'), ...
    plot(NaN,NaN,'-m')], {'Freefaces','Initial Position', ...
    'Goal Position', 'Hand Contact Points'}, 'Location','northeast');
axis([-5 5 0 10 0 10]);
view(45.7, 50);
saveas(gcf, fullfile('..', 'figures', '9free_faces_points_hand.png'))

alpha = [0;1;0;0]; % pick an alpha
t = Cone*alpha*dt; % define the twist to test

Cp_e = Cp;
Cn_e = Cn;
Rel_e = zeros(size(Cp_e,1),2);
GG_e = build_g(Cp_e, 1);

%% Robot Hand Model
Cp_h =    p_global;
Cn_h =    n_global;

GG_h = build_g(Cp_h, 1);
JJt_h = robot.get_jacobian().';

%% Robot-Environment Interaction Model
GG = [GG_h, GG_e];
JJt_e = zeros(robot.get_n_dof(),robot.get_n_contacts()*3);

%% Verify that Hand kinematics is compatible with twist
H_h = build_h(0,0,robot.get_n_contacts(),Cn_h);
c_h =  (H_h*GG_h.')*t;

G_h = (H_h*GG_h.').';
Jt_h = (H_h*JJt_h.').';

J_h = Jt_h.';
P_h = (Jt_h*J_h)\Jt_h; % left inverse of the hand jacobian
tmp = Jt_h.'*P_h;
I = eye(size(tmp));
res = truncate((I - J_h*P_h)*G_h.'*t);
% if the contact motions do not cause a motion of the hand joints
% (the hand cannot actuae that motion), then we discard the corresonding
% object twist

if norm(res) ~= 0
    disp('Twist t is not compatible with hand kinematics');
    return;
end

%% Analysis of contact point behaviour
Cp_e_m = []; % contacts to maintain
Cp_e_s = []; % contacts to maintain but sliding
Cn_e_m = []; % contacts to maintain
Cn_e_s = []; % contacts to maintain but sliding
cf_e_s_dim = [];
num_cp = 0;
c_types = [];
for i=1:size(Cp_e,1)
    disp(fprintf('Contact %d:', i));
    
    Cp_e_i = Cp_e(i,:); % get the contact i
    Cn_e_i = Cn_e(i,:);
    
    GG_e_i = build_g(Cp_e_i, 1);
    H_e_i = build_h(0,0,1,Cn_e_i);
    c_e_p = H_e_i*GG_e_i'*t;
    if(truncate(norm(c_e_p)) == 0)
        Cp_e_m = [Cp_e_m;Cp_e_i];
        Cn_e_m = [Cn_e_m;Cn_e_i];
        cf_e_s_dim = [cf_e_s_dim 3];
        num_cp = num_cp +1;
        disp('    maintained');
        c_types = [c_types; 1];
    elseif (truncate(Cn_e_i*c_e_p) > 0)
        disp('    detached');
        c_types = [c_types; 2];
    elseif (truncate(Cn_e_i*c_e_p) < 0)
        disp('    WARNING - to be debugged');
        c_types = [c_types; -1];
    else
        Cp_e_s = [Cp_e_s;Cp_e_i];
        Cn_e_s = [Cn_e_s;Cn_e_i];
        cf_e_s_dim = [cf_e_s_dim 1];
        num_cp = num_cp +1;
        disp('    slipping');
        c_types = [c_types; 3];
    end
    
end

%% Build G for sliding contacts
mu = 0.5; % ????
GesS = []; % Ges * S
for i=1:size(Cp_e_s,1)
    Cp_e_s_i = Cp_e_s(i,:);
    Cn_e_s_i = Cn_e_s(i,:);
    GG_s_i = build_g(Cp_e_s_i,1);
    H_s_i = build_h(0,0,1,Cn_e_s_i);
    c_e_p = H_s_i*GG_s_i'*t;
    % to be generalized for any contact type
    S_i = [Cn_e_s_i' - mu*c_e_p/norm(c_e_p);0;0;0];
    GesS = [GesS GG_s_i*S_i];
end


%% Build G for maintained contacts
Gem = (build_h(0,0,size(Cp_e_m,1),Cp_e_m)*build_g(Cp_e_m,1).').';

%% Build G total including Hand contacts
G = [G_h Gem GesS];

K = eye(size(G,2))*50;
Gr= K*G'*pinv(G*K*G'); % Warning: may be singular if G is not full rank

A = kernel(G);

[rowA, colA] = size(A);

J_e = zeros(size([Gem GesS],2),robot.get_n_dof());
J = [J_h;J_e]; % full system Jacobian

Q = [A  -K*J  K*(G)'];
B = kernel(Q);
E = abgram(A*B(1:colA,:)); % Base for internal controllable forces

lbd = 1; Mtd = 'Hi';
[t_unused,nDoF]=size(E);

[rA,nA] = size(A); yoo = zeros(nA,1);

we = [0;-1;0;0;0;-1]*9.81;
fc_0 = Gr*we; % External wrench

%% building data for optmization problem
normals = reshape(Cn_h', size(Cn_h,1)*3,1);
normals = [normals;reshape(Cn_e_m', size(Cn_e_m,1)*3,1)];
normals = [normals; zeros(size(Cp_e_s,1),1)];

cf_dim = [ones(1,size(Cp_h,1))*3 cf_e_s_dim];
mu_vect = [10 ones(1,num_cp)]*mu;
num_cp = num_cp + size(Cn_h,1);
f_min_vect = 0.1*ones(1,num_cp);
f_max_vect = 100*ones(1,num_cp);

% V_0 = V_tot(fc_0, normals, mu_vect, f_min_vect, f_max_vect, cf_dim ), ...
% E_el)
%
% [fc_opt, y, V_opt_mincon_1, V_0,exitflag,output, elapsed_time, ...
% sigma_leq, lambda,grad,hessian] = ...
%                                   V_optimal_mincon(fc_0, normals, ...
%                                         mu_vect, f_min_vect, ...
%                                         f_max_vect, cf_dim, E );

%% plot results

Cp = [Cp_h;Cp_e];
Cn = [Cn_h;Cn_e];
c_types = [1*ones(size(Cp_h,1),1);c_types];
figure('Color',[1 1 1]);
plot_system(box_object, Cp, Cn, t, c_types, mu, robot);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
xlabel('z');
ylabel('x');
zlabel('y');
legend([plot(NaN,NaN,'-b'), plot(NaN,NaN,'-y'), plot(NaN,NaN,'-g'), ...
    plot(NaN,NaN,'-m')], {'Initial Position', 'Final Position', ...
    'Initial Contacts', 'Sliding Contacts'}, 'Location','northeast');
saveas(gcf, fullfile('..', 'figures', '10hand_actuating_twist.png'))

% Num of maximum samples for the RRT planner
planner_parameters.n_try = 500;
% Num of iteration that have to pass until the planner asks for 
% object_state_final connection
planner_parameters.n_check = 10;
% Space dimension
planner_parameters.d = 6;
% Space step, rotation step dimensions
planner_parameters.twist_step = [0.2, 0.2];
% probability of trying straight line twist if free from contacts
planner_parameters.try_straight = 0.5;
% probability of staying on cone edge
planner_parameters.p_edge = 0.2;
% This parameter defines if the planner looks for ne hand configuration 
% on each iteration or not
planner_parameters.hold_robot_config = false;

config_to_hold.Cp =  p;
config_to_hold.Cn =  n;
planner_out = grasp_object_RRT(environment, box_object, ...
    target_position, robot, planner_parameters, config_to_hold);

figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
all_boxes = {box_shelf, box_left, box_right, box_wall, box_object};
plot_boxes(all_boxes, true);
xlabel('z');
ylabel('x');
zlabel('y');
axis([-5 5 0 10 0 10]);
view(45.7, 50);

plot_tree(planner_out)

if (planner_out.done == true)
    for i =1:length(planner_out.path_out)-1
        b = planner_out.V{planner_out.path_out(i)};
        plot_box(b.l, b.w, b.h, b.T, [0 1 0], true);
    end
    t = get_direct_twist(target_position, b, 1.0);
    for i=0.01:0.2:0.9
        
        book_moved = twist_moves_object(b, t*i);
        plot_box(book_moved.l, book_moved.w, book_moved.h, ...
            book_moved.T, [0 1 0], true)
        
    end
    
    plot_box(target_position.l, target_position.w, target_position.h, ...
        target_position.T, [0 1 0], true);
    
end

plot_box(target_position.l, target_position.w, target_position.h, ...
    target_position.T, [0 0 0], true);

legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-g'), ...
    plot(NaN,NaN,'-k')], {'Environment','Tree', 'Final Path', ...
    'Goal Configuration'}, 'Location','northeast');

saveas(gcf, fullfile('..', 'figures', '11finalpath.png'))

figure('Color',[1 1 1], 'pos',[10 10 1000 1000]);
plot_boxes(all_boxes, true);
xlabel('z');
ylabel('x');
zlabel('y');
axis([-8 8 -6 9 -1 10]);
view(123.3, 46);

if (planner_out.done == true)
    for i =1:length(planner_out.path_out)-1
        b = planner_out.V{planner_out.path_out(i)};
        plot_box(b.l, b.w, b.h, b.T, [0 1 0], true);
    end
    t = get_direct_twist(target_position, b, 1.0);
    for i=0.01:0.2:0.9
        
        book_moved = twist_moves_object(b, t*i);
        plot_box(book_moved.l, book_moved.w, book_moved.h, ...
            book_moved.T, [0 1 0], true)
        
    end
    
    plot_box(target_position.l, target_position.w, target_position.h, ...
        target_position.T, [0 1 0], true);
end

plot_box(target_position.l, target_position.w, target_position.h, ...
    target_position.T, [0 0 0], true);

legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-g'), ...
    plot(NaN,NaN,'-k')], {'Environment','Tree', 'Final Path', ...
    'Goal Configuration'}, 'Location','northeast');

saveas(gcf, fullfile('..', 'figures', '12finalpath_only.png'))