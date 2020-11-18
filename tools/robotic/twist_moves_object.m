function new_object_state = twist_moves_object(object_state, t)
%TWISTMOVESOBJECT This function applies a twist to the objec in object
% state at the contact point defined by the end-effector of the robot_state
    g = twistexp(t);
    new_object_state = build_box(object_state.l, object_state.w, ...
                                object_state.h, g*object_state.T);
end