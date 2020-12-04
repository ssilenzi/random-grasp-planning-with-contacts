function plot_environment(list_boxes, filled)

if ~exist('filled','var')
  filled = false;
end

n_boxes = length(list_boxes) ;
for i= 1:n_boxes
    
    box = list_boxes{i};
    plot_box(box.l, box.w, box.h, box.T, [1 0 0], filled);
    hold on
end

end