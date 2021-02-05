function tot_h = plot_scenario(environment,box_start, ...
    box_target,axis_range,azim,elev)

% PLOT SCENARIO 

hold on;

env_h = plot_boxes_custom(environment, [1 0 0], true);
start_h = plot_box(box_start.l, box_start.w, box_start.h, ...
    box_start.T, [0 0 1], true);
target_h = [];
target_h = plot_box(box_target.l, box_target.w, box_target.h, ...
    box_target.T, [0 0 0], true);

legend([plot(NaN,NaN,'-r'),plot(NaN,NaN,'-b'),plot(NaN,NaN,'-k')],...
    {'Environment','Initial Position', 'Goal Position'},...
    'Location','northeast');
legend off

axis(axis_range) % Change the axis and view
view(azim, elev);
axis equal

tot_h = {env_h, start_h, target_h};

end

