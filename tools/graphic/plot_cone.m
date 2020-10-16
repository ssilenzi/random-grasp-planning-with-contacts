function plot_cone(environment, object, cone, twist_step, contacts)
% PLOTCONE This function plots the motions described in the "cone" of the
% "object" that is contacting the "environment"

if ~exist('twist_step','var')
    twist_step = 1;
end
if ~exist('contacts','var')
    contacts = {};
end
n_boxes = length(environment)+1;
all_boxes =  cell(n_boxes);

for i=1:length(environment)
    all_boxes{i} = environment{i};
end

all_boxes{n_boxes} = object;

for i=1:size(cone,2)
    figure('Color',[1 1 1], 'Position',[10 10 1000 1000]);
    str = sprintf('Twist: [%f %f %f %f %f %f]',cone(3,i),cone(1,i),...
        cone(2,i),cone(6,i),cone(4,i),cone(5,i));
    title(str);
    axis equal;
    view(45.7, 50);
    grid off
    boxn = twist_moves_object(object, cone(:,i)*twist_step);
    % boxn = twist_moves_object(object, cone(:,i)*twist_step,[]);
    all_boxes{n_boxes} = boxn;
    plot_boxes(all_boxes, true);
    if ~isempty(contacts)
        plot_contacts(contacts{1}, contacts{2});
    end
    xlabel('z');
    ylabel('x');
    zlabel('y');
    axis equal
end
end