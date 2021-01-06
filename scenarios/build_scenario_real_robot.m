function [box_obj,box_fin,env,boxes_all,robot,ax_range,az,el] = ...
    build_scenario_real_robot(scenario_name,robot_name)
% BUILD SCENARIO REAL ROBOT Builds the environment, the box (init and fin)
%   Inputs:
%   scenario_name	- string with the name of the m file with the scenario
%   robot_name      - string with the name of the robot class (not .m)
%   Outputs:
%   box_obj         - initial pose of the object (box struct)
%   box_fin         - final desired pose of the object (box struct)
%   env             - evironment, a list of boxes
%   boxes_all       - the list of all boxes (environment and object)
%   robot           - the robot object
%   axis_range      - axis extrema values for plots
%   azim, elev      - azimut and elevation for plots

% Build the scenario and the box (only initial pose)
% box_object, target_positionenvironment,all_boxes are set in here
run(scenario_name); 
box_obj = box_object;
box_fin = target_position;
env = environment;
boxes_all = all_boxes;
ax_range = axis_range;
az = azim;
el = elev;

% Loading the hand
robot = load_gripper(robot_name);

% Creating the first figure with robot started
fig_h = figure('Color',[1 1 1], 'pos',[0 0 800 800], ...
    'WindowState', 'maximized');
rob_h = robot.plot();
tot_h = plot_scenario(env, box_obj, box_fin, ax_range, az, el);

end

