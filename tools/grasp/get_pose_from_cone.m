function [success,new_box_obj,twist,d_pose] = get_pose_from_cone(Cone,box_obj,environment,dt,alpha,n_try)
% GET POSE FROM CONE - Samples a pose from free motions cone
%   Inputs:
%   Cone        - convex basis of the free motions cone
%   box_obj     - box object with all properties
%   environment - the set of boxed making the environment
%   dt          - max time interval for pose from cone gen. velocity
%   alpha       - combination vector (if not provided, random sampling)
%   n_try       - number of tries for getting a new collision free pose
%   Outputs:
%   success     - bool for success
%   new_box_obj - new object pose
%   twist       - twist from box_obj to new_box_obj
%   d_pose      - variation of pose from box_obj to new_box_obj

verbose = false;

[~,cC] = size(Cone);

% If alpha not provided select a random combination vector
if ~exist('alpha','var')
    alpha = rand(cC,1);
end

% If alpha not provided select a random combination vector
if ~exist('n_try','var')
    n_try = 50;
end

% Getting the twist
twist = Cone*alpha;
twist = twist/norm(twist); % normalizing

% Getting a collision free pose variationM and moving the box
t_range = dt:-0.01:0;
success = false;
new_box_obj = box_obj;
d_pose = twist;
for i = 1:length(t_range)
    % The rand is multiplied below in order to not have the same d_pose 
    % always for the same cone (0.4*rand + 0.6) = random val in (0.6, 1)
    d_pose = twist*t_range(i)*(0.4*rand + 0.6); 
    new_box_obj = twist_moves_object(box_obj, d_pose);
    
    % Checking the object env collision
    [bool, coll_type] = check_collisions_box(new_box_obj, environment);
    if bool == true
        if verbose
            fprintf('box %s collision\n', coll_type)
        end
    else
        if verbose
            disp('box no collision\n');
        end
        success = true;
        break;
    end
end

end

