function [box_obj,box_fin,env,boxes_all,robot,ax_range,az,el] = ...
    build_scenario(scenario_name,robot_name,link_dims,is_und,we)
% BUILD SCENARIO Builds the environment, the box (initial and final)
%   Inputs:
%   scenario_name	- string with the name of the m file with the scenario
%   robot_name      - string with the name of the robot class (not .m)
%   link_dims       - dimensions of the links of the robot
%   is_und          - flag for underactuation
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

% Change the axis and view
axis(ax_range); 
axis equal;
view(az, el);
legend off;

% Plot the external wrench (gravity?)
% plot_forces([0 10 0], we.');

% Loading the hand
robot = load_gripper(robot_name,link_dims,is_und);

end

