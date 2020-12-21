%% TEST - Robot Loading and Matrices %%

clear all; clc;

% Load and show the robot
panda = loadrobot("frankaEmikaPanda");
show(panda);
showdetails(panda);

% Getting some basic info
home_config = homeConfiguration(panda);
num_joints = numel(home_config);
ee_1_name = "panda_leftfinger";
ee_2_name = "panda_rightfinger";
wrist_name = "panda_hand";

% Getting the fk and jacobian to ee and wrist in rand. config.
rand_config = randomConfiguration(panda);

x_1 = getTransform(panda, rand_config, ee_1_name);          % left
jac_1 = geometricJacobian(panda, rand_config, ee_1_name);

x_2 = getTransform(panda, rand_config, ee_2_name);          % right
jac_2 = geometricJacobian(panda, rand_config, ee_2_name);

x_w = getTransform(panda, rand_config, wrist_name);          % wrist
jac_w = geometricJacobian(panda, rand_config, wrist_name);

% Trying the ik algorithms
task_w_1 = trvec2tform([0.4 0 0.2])*axang2tform([0 1 0 pi]);

ik = inverseKinematics('RigidBodyTree', panda);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];
joint_config_1 = ik(wrist_name, task_w_1, weights, home_config);

show(panda, joint_config_1);
