function [success,new_box_obj,twist,d_pose] = get_pose_from_cone(Cone,box_obj,environment,dt,alpha)
% GET POSE FROM CONE - Samples a pose from free motions cone
%   Inputs:
%   Cone        - convex basis of the free motions cone
%   box_obj     - box object with all properties
%   environment - the set of boxed making the environment
%   dt          - max time interval for pose from cone gen. velocity
%   alpha       - combination vector (if not provided, random sampling)
%   Outputs:
%   success     - bool for success
%   new_box_obj - new object pose
%   twist       - twist from box_obj to new_box_obj
%   d_pose      - variation of pose from box_obj to new_box_obj

% ATTENTION! dt should be small (e.g. 0.5) otherwise object could pass
% through walls.
% TODO: change this by checking for collision all the way but maybe this
% would slow down everyting

verbose = false;
exhaustive_search = false;

[~,cC] = size(Cone);

% If alpha not provided select a random combination vector
if ~exist('alpha','var')
    alpha = rand(cC,1);
end

% Getting the twist
twist = Cone*alpha;
twist = twist/norm(twist); % normalizing

% Getting a collision free pose variation and moving the box
t_range = dt:-0.01:0;
success = false;
new_box_obj = box_obj;
d_pose = twist;
best_dt = 0;

% Now we check from max dt to 0 which dt gives a collision free movement:
% all the different dts shall be checked to avoid the issue of passing
% though walls
for i = 1:length(t_range)
    
    % Variation with the present dt
    d_pose = twist*t_range(i);
    new_box_obj = twist_moves_object(box_obj, d_pose);
    
    % Checking the object env collision
    [bool, coll_type] = check_collisions_box(new_box_obj, environment);
    if bool == true
        if verbose
            fprintf('box %s collision\n', coll_type)
        end
        best_dt = 0;
    else
        if verbose
            disp('box no collision\n');
        end
        if exhaustive_search && t_range(i) > best_dt
            best_dt = t_range(i);
            continue;
        else
            best_dt = t_range(i);
            break;
        end
    end
end

if verbose
    fprintf('Found best_dt is %d', best_dt)
end

if best_dt == 0
    return;
end

% If here found a good pose
success = true;

% Building the new obj pose and d_pose with the best dt
% The rand is multiplied below in order to not have the same d_pose 
% always for the same cone (0.9*rand + 0.1) = random val in (0.9, 1)
d_pose = twist*best_dt*(0.8*rand + 0.1);
new_box_obj = twist_moves_object(box_obj, d_pose);

end

