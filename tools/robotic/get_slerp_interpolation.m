function [hom_seq,hom_glob,q_slerp,p_seq] = get_slerp_interpolation(object_state_final,object_state, ...
    seq_vec)

% GET SLERP INTERPOLATION - Retruns a sequence of hom. mats of a slerp
% interpolation to connect "object_state" to object_state_final".
% seq_vec is an array of equally spaced interpolation coefficients (e.g.
% between 0 and 1)

% Getting the quaternions from the two orientations
q1 = rotm2quat(object_state.T(1:3,1:3));
q2 = rotm2quat(object_state_final.T(1:3,1:3));

% Getting the slerp interpolation
q_slerp = zeros(4,length(seq_vec));
for k = 1:length(seq_vec)
        q_slerp(:,k) = quatinterp(q1,q2,seq_vec(k),'slerp').'; % in columns
end

% Getting the linear translation sequence
p1 = object_state.T(1:3,4);
p2 = object_state_final.T(1:3,4);
p_seq = zeros(3,length(seq_vec));
for k = 1:length(seq_vec)
    p_seq(:,k) = (1 - seq_vec(k))*p1 + seq_vec(k)*p2; % in columns
end

% Getting the corresponding homogeneous matrices
hom_glob = zeros(4,4,length(seq_vec));
for k = 1:length(seq_vec)
    hom_tmp = [quat2rotm(q_slerp(:,k).'), p_seq(:,k);
        zeros(1,3), 1];
    hom_glob(:,:,k) = hom_tmp;
end

% Computing the transforms to be applied in sequence to get the motion
hom_seq = zeros(4,4,length(seq_vec));
hom_seq(:,:,1) = hom_glob(:,:,1)*inv(object_state.T);
for k = 2:length(seq_vec)
    hom_tmp = hom_glob(:,:,k)*inv(hom_glob(:,:,k-1));
    hom_seq(:,:,k) = hom_tmp;
end

end