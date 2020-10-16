function resolve_paths
current_file = mfilename('fullpath');
[base_path,~,~] = fileparts(current_file);
if ispc
    sep = '\';
else
    sep = '/';
end
base_path = split(base_path, sep);
main_path = base_path;
main_path{end} = 'main';
tools_path = base_path;
tools_path{end} = 'tools';
main_path = join(main_path, sep);
tools_path = join(tools_path, sep);
main_path = main_path{:};
tools_path = tools_path{:};
path_cell = regexp(path, pathsep, 'split');
if ispc
    on_path = any(strcmpi(main_path, path_cell));
else
    on_path = any(strcmp(main_path, path_cell));
end
if ~on_path
    addpath(main_path);
    addpath(tools_path);
    addpath(strcat(tools_path, sep, 'graphic'));
    addpath(strcat(tools_path, sep, 'grasp'));
    addpath(strcat(tools_path, sep, 'robotic'));
    addpath(strcat(tools_path, sep, 'test'));
end
end