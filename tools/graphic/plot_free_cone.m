function plot_free_cone(Cone,dt,box_obj,all_boxes,axis_range,az,el)
% PLOT FREE CONE - Plots the generators of the given free motions cone
%   Inputs:
%   Cone        - convex basis of the free motions cone
%   dt          - time interval for pose from cone gen. velocity
%   box_obj     - box object with all properties
%   all_boxes   - array with all boxes (environment and object)
%   axis_range  - vector with the x y z ranges
%   az, el      - azimut and elevation for view

for i=1:size(Cone,2)
    figure('Color',[1 1 1],'Position',[10 10 1000 1000]);
    str = sprintf('Twist: [%f %f %f %f %f %f]',Cone(3,i),Cone(1,i),...
        Cone(2,i),Cone(6,i),Cone(4,i),Cone(5,i));
    title(str)
%     axis(axis_range);
    axis equal;
    view(az, el);
    grid off
    boxn = twist_moves_object(box_obj,Cone(:,i)*dt);
    plot_boxes(all_boxes, true);
    plot_box(boxn.l, boxn.w, boxn.h, boxn.T, [0 0 0], true);
    xlabel('z');
    ylabel('x');
    zlabel('y');
end

end

