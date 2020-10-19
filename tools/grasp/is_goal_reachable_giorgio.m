function [res, id] =  is_goal_reachable_giorgio(G_tree, object_state_final)
% ISGOALREACHABLE This function sould return yes if the object is already
% portable to object_state_final

for i = G_tree.n_nodes:-1:1
    t = get_direct_twist(object_state_final, G_tree.V{i});
    if norm(t) < 1e-3
        id = i;
        res = true;
        return
    end
end

id = 0;
res = false;
end