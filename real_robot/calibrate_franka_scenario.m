function calibrate_franka_scenario(franka)

rosshutdown
rosinit('172.16.0.6');

% Creating subscribers and tf echoer
joint_states_sub = rossubscriber('/joint_states');
tftree = rostf;

exit = 0;
while ~exit
    
    disp('Put the robot in a pose!');
    pause;
    
    % Getting needed info
    joints_msg = receive(joint_states_sub, 5);
    q_arr = joints_msg.Position;
    Tp1 = getTransform(tftree, 'world', 'panda_leftfinger_tip');
    cp1 = Tp1.Transform.Translation;
    Tp2 = getTransform(tftree, 'world', 'panda_rightfinger_tip');
    cp2 = Tp2.Transform.Translation;
    Cp = [cp1.X, cp1.Y, cp1.Z;
        cp2.X, cp2.Y, cp2.Z];    
    
    % Creating the next robot
    franka_f = copy(franka);
    franka_f.set_config(q_arr);
    franka_f.plot([], false, gca);
%     plot3(Cp(:,1), Cp(:,2), Cp(:,3), 'r*')

    % Exit?
    disp('Exit? [1] yes, [0] no.');
    exit = input(prompt);
    if (exit ~= 1 && exit ~= 0)
        error('You did not input a correct number!');
    end
    
end

end

