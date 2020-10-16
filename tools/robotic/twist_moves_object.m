function new_object_state = twist_moves_object(object_state, t)
% TWISTMOVESOBJECT This function applies a twist to the objec in object
% state at the contact point defined by the end-effector of the robot_state
    g = twistexp(t);
    new_object_state =  object_state;
    new_object_state.T = g*object_state.T;
end