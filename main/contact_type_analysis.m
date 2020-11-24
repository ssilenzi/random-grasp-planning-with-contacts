function [Cp_e_out,Cn_e_out,cf_e_dim,c_types] = contact_type_analysis(Cp_e,Cn_e,d_pose)
% GET CONTACT TYPES - From the present contacts and the object pose
% variation get the type of contacts and remove the detached ones
%   Inputs:
%   Cp_e        - environment contact positions (rows)
%   Cn_e        - environment contact normals (rows)
%   d_pose      - variation of object pose
%   Outputs:
%   Cp_e_out    - environment contact positions (rows) without detached
%   Cn_e_out    - environment contact normals (rows) without detached

% Getting the type of contacts and f vectors dimensions
cf_e_dim = [];  % Dimensions of the force vector
c_types = [];   % Type (1 - maint, 2 - det, 3 - ?, 4 - slide)
Cp_e_out = Cp_e;
Cn_e_out = Cn_e;
for i=1:size(Cp_e,1)
    disp(fprintf('Contact %d:', i));
    
    Cp_e_i = Cp_e(i,:); % get the contact i
    Cn_e_i = Cn_e(i,:);
    
    GG_e_i = build_g(Cp_e_i, 1);
    H_e_i = build_h(0,0,1,Cn_e_i);
    c_e_p_i = H_e_i*GG_e_i'*d_pose;
    if(truncate(norm(c_e_p_i)) == 0)
        cf_e_dim = [cf_e_dim 3];
        disp('    maintained');
        c_types = [c_types; 1];
    elseif (truncate(Cn_e_i*c_e_p_i) > 0)
        disp('    detached');
        c_types = [c_types; 2];
    elseif (truncate(Cn_e_i*c_e_p_i) < 0)
        disp('    WARNING - to be debugged - cont-env compenetration?');
        c_types = [c_types; -1];
    else
        cf_e_dim = [cf_e_dim 1];
        disp('    slipping');
        c_types = [c_types; 3];
    end
    
end

% Removing the detached indexes from c_types and related contact normal and position
indexes_det = find(c_types == 2);
Cp_e_out(indexes_det,:) = [];
Cn_e_out(indexes_det,:) = [];
c_types(indexes_det) = [];

end

