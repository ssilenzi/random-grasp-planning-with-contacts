function robot_state = generate_robot_config(robot, Cp, Cn)
% GENERATEROBOTCONFIG This function returns a configuration of the "robot"
% structure that contacts the "object_state" which is constrained by the 
% contact points "object_state.Cn" and their normals "object_state.Cn"

% 3- Get a hand configuration
q = robot.getStartingConfig(Cp, Cn);
robot.setConfig(q);
% robot.plot();

xd = zeros(4,4,robot.getnContacts()+1);
for i =1:robot.getnContacts()
    xd(:,:,i) = [eye(3) Cp(i,1:3)';[0 0 0 1]];
end
xd(:,:,robot.getnContacts()+1) = [eye(3) [0 0 0]';[0 0 0 1]];
% this should generate wrist desired wrist position 
  
robot.computeDiffIK(xd,[1,1], 1/5);

robot_state = robot;
end