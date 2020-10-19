function [t, index] = sample_twist_from_tree(G, object_state_final, ...
    twist_step, try_straight, p_edge)
% Sample a cone

index = 1 + round(rand()*(G.n_nodes-1));

[res, id] = is_goal_reachable_giorgio(G, object_state_final);

if  res && (rand()<try_straight)
    disp('trying straight motion')
    t = get_direct_twist(object_state_final, G.V{id});
else    
    t = sample_twist_from_cone(G.C{index}, p_edge);   
end
t = [twist_step(1)*eye(3), zeros(3); zeros(3), twist_step(2)*eye(3)]*t;
end