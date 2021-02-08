function handle_boxes = plot_boxes(list_boxes, filled, is_all)

if ~exist('filled','var')
  filled = false;
end

if ~exist('is_all','var')
  is_all = true;
end

handle_boxes = {};

n_boxes = length(list_boxes) ;
if is_all
    len_boxes = n_boxes - 1;
else
    len_boxes = n_boxes;
end

for i= 1:len_boxes
    
    box = list_boxes{i};
    handle_boxes{i} = plot_box(box.l, box.w, box.h, box.T, [1 0 0], filled);
    hold on
end

if is_all
    box = list_boxes{n_boxes};
    handle_boxes{n_boxes} = plot_box(box.l, box.w, box.h, box.T, [0 0 1], filled);
end

end