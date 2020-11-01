function gripper = load_gripper(name)

if strcmp(name, 'hand_example')
    gripper = hand_example();
    return;
end

error('Gripper not in database');
end