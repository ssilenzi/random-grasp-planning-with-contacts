function handle = plot_boxes(list_boxes, filled)

if ~exist('filled','var')
  filled = false;
end

n_boxes = length(list_boxes);
handle = {};
for i = 1:n_boxes-1
    box = list_boxes{i};
    handle{i} = plot_box(box.l, box.w, box.h, box.T, [1 0 0], filled);
    hold on
end

box = list_boxes{n_boxes};
handle{n_boxes} = plot_box(box.l, box.w, box.h, box.T, [0 0 1], filled);
end