function gripper = load_gripper(name, link_dimensions, is_und)

if strcmp(name, 'hand_example')
    if (~exist('link_dimensions', 'var') || ~isequal([4,1], ...
            size(link_dimensions)))
        link_dimensions = ones(4,1);
    end
    if (~exist('is_und', 'var'))
        is_und = false;
    end
    gripper = hand_example(link_dimensions,is_und);
    return;
end

error('Gripper not in database');
end