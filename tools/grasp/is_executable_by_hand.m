function [success,y_star,dq_star,du_star] = ...
    is_executable_by_hand(fc_opt,we,cf_dim_tot,G,J,K,Delta)

% IS EXECUTABLE BY HAND Checks if force executable by hand

scale = 1e-8;
success = false;
dq_star = [];
du_star = [];

ind_h = cf_dim_tot(1)+cf_dim_tot(2);
[E, dQ, dU] = basis_active_internal_forces_2(G, J, K);
fc_hand = fc_opt - pinv(G)*we;
fc_hand = fc_hand(1:ind_h);
E = E(1:ind_h,:);
y_star = linsolve(E,fc_hand);
% norm(fc_hand - E*y_star)
if (norm(fc_hand - E*y_star) < Delta)
    success = true;
    dq_star = scale*dQ*y_star;
    du_star = scale*dU*y_star;
end

end

