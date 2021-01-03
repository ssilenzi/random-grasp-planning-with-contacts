function ax = frankaVisualizeCollisionEnvironment(collisionObjectArray)
% frankaVisualizeCollisionEnvironment Visualize a set of collision objects
%   given a cell array of collision objects on the current figure

% Get axis properties and set hold
ax = gca;
hold all;

% Show remaining objects
for i = 1:numel(collisionObjectArray)
    show(collisionObjectArray{i}, "Parent", ax);
end

% Set axis properties
% axis equal;

end

