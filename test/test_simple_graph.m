%% Basic graph test

% Initialize a starting node
G = digraph([],[]);
pose_init = rand(4,4);
NewNode = table({pose_init}', 'VariableNames', {'Pose'});
G = addnode(G,NewNode)

plot(G);

% Adding a node
pose_init2 = rand(4,4);
NewNode = table({pose_init2}', 'VariableNames', {'Pose'});
G = addnode(G,NewNode)

plot(G);

% Adding a labeled and weighted edge
NewEdges = table({'moving'}', [10]', ...
    'VariableNames',{'Type','Weight'});
G = addedge(G,1,2,NewEdges)

plot(G);