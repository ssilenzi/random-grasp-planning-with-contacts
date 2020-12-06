function gripper = load_gripper(name, link_dimensions)

if strcmp(name, 'hand_example')
    if (~exist('link_dimensions', 'var') || ~isequal([4,1], ...
            size(link_dimensions)))
        link_dimensions = ones(4,1);
    end
    gripper = hand_example(link_dimensions);
    return;
end

error('Gripper not in database');
end