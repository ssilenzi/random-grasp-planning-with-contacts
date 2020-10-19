function [res, id] =  is_goal_reachable(environment, G_tree, ...
    object_state_final)
% ISGOALREACHABLE This function sould return yes if the object is already
% portable to object_state_final with out collission

for i = G_tree.n_nodes:-1:1
    t = get_direct_twist(object_state_final, G_tree.V{i});
    for j=0:0.2:1
        boxn = twist_moves_object(G_tree.V{i}, t*j);
        res_local = true;
        if(is_box_in_collision(environment, boxn))
            res_local = false;           
            break;
        end
    end
    if res_local
        id = i;
        res = true;
        return;
    end
end

id = 0;
res = false;
end