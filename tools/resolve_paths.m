function resolve_paths
current_file = mfilename('fullpath');
[base_path,~,~] = fileparts(current_file);
if ispc
    sep = '\';
else
    sep = '/';
end
base_path = split(base_path, sep);
grippers_path = base_path;
grippers_path{end} = 'grippers';
main_path = base_path;
main_path{end} = 'main';
scenarios_path = base_path;
scenarios_path{end} = 'scenarios';
optimization_path = base_path;
optimization_path{end} = 'optimization';
tools_path = base_path;
tools_path{end} = 'tools';
tree_path = base_path;
tree_path{end} = 'tree';
grippers_path = join(grippers_path, sep);
main_path = join(main_path, sep);
scenarios_path = join(scenarios_path, sep);
optimization_path = join(optimization_path, sep);
tools_path = join(tools_path, sep);
tree_path = join(tree_path, sep);
grippers_path = grippers_path{:};
main_path = main_path{:};
scenarios_path = scenarios_path{:};
optimization_path = optimization_path{:};
tools_path = tools_path{:};
tree_path = tree_path{:};
path_cell = regexp(path, pathsep, 'split');
if ispc
    on_path = any(strcmpi(main_path, path_cell));
else
    on_path = any(strcmp(main_path, path_cell));
end
if ~on_path
    addpath(grippers_path)
    addpath(strcat(grippers_path, sep, 'franka_collision_check'))
    addpath(main_path)
    addpath(scenarios_path)
    addpath(strcat(scenarios_path, sep, 'franka'))
    addpath(optimization_path)
    addpath(tools_path)
    addpath(strcat(tools_path, sep, 'graphic'))
    addpath(strcat(tools_path, sep, 'grasp'))
    addpath(strcat(tools_path, sep, 'gripper'))
    addpath(strcat(tools_path, sep, 'robotic'))
    addpath(strcat(tools_path, sep, 'test'))
    addpath(tree_path)
end
end