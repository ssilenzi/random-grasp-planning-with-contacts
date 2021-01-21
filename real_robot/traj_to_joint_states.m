function traj_to_joint_states(joint_states,traj_point)
% TRAJ TO JOINT STATES - Converts a trajectory_msgs/JointTrajectoryPoint to
% a sensor_msgs/JointStates

joint_states.Position = traj_point.Positions;
joint_states.Velocity = traj_point.Velocities;

end

