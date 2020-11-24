function [success] = is_compatible_motion_hand_kin(robot,Cp_h,Cn_h,d_pose)
% IS COMPATIBLE MOTION HAND KIN - Checks if chosen motion is compatible
% with hand kinematics
%
% Find the motion of the contact points of the hand caused by the object
% pose var. and check if it is within the range of HJ (actuatable using q).
% If the contact motions do not cause a motion of the hand joints
% (the hand cannot actuate that motion), then throw a warning and return
% false
%
%   Inputs: 
%   robot      	- robot with all its props and funcs (to get jacobian)
%   Cp_h        - hand contact positions (rows)
%   Cn_h        - hand contact normals (rows)
%   d_pose      - variation of object pose
%   Output:
%   success     - boolean true if obj. motion is compatible with hand kin.

H_h = build_h(0,0,size(Cp_h,1),Cn_h); % hard finger
G_h = build_g(Cp_h, 1);
J_h = robot.get_jacobian();

% Motion of contact points caused by object pose variation
dc_h =  H_h * G_h.' * d_pose;
% disp('dc_h '); disp(dc_h);

% Checking if dc_h can be done by hand jacobian
HJ_h = H_h * J_h;
if rank(HJ_h) ~= rank([HJ_h, dc_h])
    warn('Twist is not compatible with hand kinematics');
    success = false;
else
    success = true;
end

end

