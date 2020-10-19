% Questo file deve essere eliminato


function gripper = load_gripper(name)

if strcmp(name, 'hand_example')
    gripper = hand_example();
    return;
end

disp('Gripper not in database');
end