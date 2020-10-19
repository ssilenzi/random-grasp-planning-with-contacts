function path_indexes = extract_path(G)
% EXTRACTPATH This function returns a path fom the tree "G" if it exists,
% an empty array is returned otherwise.

if (~G.done)
    return;
end

path_indexes_fliped = G.E(end,2);
i = find(G.E(:,2) == path_indexes_fliped);
while G.E(i,1)~=1
   path_indexes_fliped = [path_indexes_fliped, G.E(i,1)];
   i = find(G.E(:,2) == path_indexes_fliped(end));
end
path_indexes_fliped = [path_indexes_fliped 1];

path_indexes = fliplr(path_indexes_fliped);
end