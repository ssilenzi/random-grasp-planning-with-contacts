function t = get_direct_twist(object_state_final, object_state, twist_step)
% GETDIRECTTWIST This function returns a twist that describes the velocity
% to connect "object_state" to object_state_final". If "twist_step" is
% provided the twist is scaled, otherwise the twist is such that applyed to
% object_state it arrives to object_state_final

if ~exist('twist_step', 'var')
    twist_step = 1;
end

% relative transformation to state final in state initail
T_i_f= inv(object_state.T)*object_state_final.T;
[t, alpha] = homtotwist(T_i_f)
ad(object_state.T)
t = ad(object_state.T)*t*alpha*twist_step

end