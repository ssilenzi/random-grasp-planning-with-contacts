function plot_system(Box, Cp, Cn,t, c_types, mu, robot, axis_dim, view_exp)

if ~exist('axis_dim','var')
    axis_dim = [-5 5 0 10 0 10];
end
if ~exist('view_exp','var')
    view_exp_a = 45.7;
    view_exp_b = 50;
else
    view_exp_a = view_exp(1);
    view_exp_b = view_exp(2);
end

hold on
ax1 = subplot(1,2,1);
plot_boxes({Box}, true);
plot_contacts(Cp, Cn)
xlabel('z');
ylabel('x');
zlabel('y');
if norm(t) > 0
    num_contacts = size(Cp,1);
    G = (build_h(0,0,num_contacts,Cn)*build_g(Cp,1).').';
    moved_points = reshape(G.'*t, [3, num_contacts]);
    Cp_n = Cp + moved_points';
    % box_center_n = Box.T(1:3,4) + twist_moves_point(Box.T(1:3,4), t);
    plot3(  Cp_n(:,3), Cp_n(:,1), Cp_n(:,2),'g*')
    boxm = twist_moves_object(Box, t) ;
    plot_box(boxm.l, boxm.w, boxm.h, boxm.T, [1 1 0], true);
end
robot.plot();
% axis equal
axis(axis_dim);
% axis equal
view(view_exp_a, view_exp_b);
str = sprintf('Twist: [%f %f %f %f %f %f]',t(3),t(1),t(2),t(6),t(4),t(5));
title(str);

ax2 = subplot(1,2,2);
plot_boxes({Box}, true);
hold on
xlabel('y')
ylabel('z')
zlabel('x')
if norm(t) > 0
    num_contacts = size(Cp,1);
    G = (build_h(0,0,num_contacts,Cn)*build_g(Cp,1).').';
    moved_points = reshape(G.'*t, [3, num_contacts]);
    Cp_n = Cp + moved_points.';
    boxm = twist_moves_object(Box, t);
    Cn_n = Cn * boxm.T(1:3,1:3).';
    boxm = twist_moves_object(Box, t);
    plot_box(boxm.l, boxm.w, boxm.h, boxm.T, [1 1 0], true);
end
plot_contacts_types(Cp_n, Cn_n, c_types, mu, t, [1 0 1]);
% axis equal
axis(axis_dim);
% axis equal
view(view_exp_a, view_exp_b);
% linkaxes([ax1,ax2],'xy')
Link = linkprop([ax1, ax2],{'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);

g = twistexp(t);
T_all = robot.getAllT();
T_new_wirst = g*T_all(:,:,6);
q = robot.q;
q(1:6,1) = [T_new_wirst(1:3,4)' rotm2eul(T_new_wirst(1:3,1:3))];
robot.setConfig(q);
robot.plot();
str = sprintf('Twist: [%f %f %f %f %f %f]',t(3),t(1),t(2),t(6),t(4),t(5));
title(str);

end