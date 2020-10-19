function plot_tree(G)
for  i=1:G.n_nodes
    plot_box(G.V{i}.l, G.V{i}.w, G.V{i}.h, G.V{i}.T, ...
        [0, 1-(i/G.n_nodes), 1]);
end
alpha(0.3)
end