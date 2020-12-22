function handle_boxes = plot_boxes_custom(list_boxes, RGBColor, filled)

if ~exist('RGBColor','var')
  RGBColor = [1 0 0];
end
if ~exist('filled','var')
  filled = false;
end

n_boxes = length(list_boxes) ;
handle_boxes = cell(1,n_boxes);

for i= 1:n_boxes
    
    box = list_boxes{i};
    handle_boxes{i} = plot_box(box.l, box.w, box.h, box.T, RGBColor, filled);
    hold on
end

end