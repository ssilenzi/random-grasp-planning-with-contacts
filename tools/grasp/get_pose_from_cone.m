function [new_box_obj,twist,d_pose] = get_pose_from_cone(Cone,box_obj,dt,alpha)
% GET POSE FROM CONE - Samples a pose from free motions cone
%   Inputs:
%   Cone        - convex basis of the free motions cone
%   box_obj     - box object with all properties
%   dt          - time interval for pose from cone gen. velocity
%   alpha       - combination vector (if not provided, random sampling)
%   Outputs:
%   new_box_obj - new object pose
%   twist       - twist from box_obj to new_box_obj
%   d_pose      - variation of pose from box_obj to new_box_obj

[~,cC] = size(Cone);

% If alpha not provided select a random combination vector
if ~exist('alpha','var')
    alpha = rand(cC,1);
end

% Getting the twist and the pose variation
twist = Cone*alpha;
twist = twist/norm(twist);
d_pose = twist*dt;

% Moving the box to get the new box object
new_box_obj = twist_moves_object(box_obj, d_pose);

end

