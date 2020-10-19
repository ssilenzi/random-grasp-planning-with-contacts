% This is an example representing a book constrained by its front and back
% cover, spine and lower side
close all
clear
run(fullfile('..', 'tools', 'resolve_paths.m'))

Cp = [  0 3 0
        3 3 0
        3 3 5
        0 3 5
        0 4 0
        3 4 0
        3 4 5
        0 4 5
        0 3 0
        0 4 0
        3 3 0
        3 4 0 
        0 3 0
        0 3 5
        0 4 5
        0 4 0
        ];


Cn =  [ 0 1 0
        0 1 0
        0 1 0
        0 1 0
        0 -1 0
        0 -1 0
        0 -1 0
        0 -1 0
        0 0 1
        0 0 1
        0 0 1
        0 0 1
        1 0 0
        1 0 0
        1 0 0
        1 0 0
        ]; 

T = eye(4); 
T(1:3,4) = [1.5 3.5 2.5].';
box = build_box(3,1,5,T);

%% Perform Partial From Closure
figure 
hold on
plot_boxes({box}, true);
plot_contacts(Cp, Cn);
plot_csys(box.T, 1);
cone = pfc_analysis(Cp, Cn, 6);
plot_cone({box},box,cone, 1, {Cp, Cn});