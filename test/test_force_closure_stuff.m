%% For testing force-closure both with the environment and the hand
% Here we will test force-closure without sliding contacts

test_hand_functions;

% Resaving needed contact matrices
Cp_e = Cp;          % Contact positions of env to obj
Cn_e = Cn;          % Contact normals of env to obj
Cp_h = p_global;    % Contact positions of hand to obj
Cn_h = n_global;    % Contact normals of hand to obj

%% Hand + environment force-closure

% Building needed matrices
