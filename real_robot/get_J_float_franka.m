function [Jm,Jf] = get_J_float_franka(franka,J01,Cp_h,Cn_h)

J = franka.get_jacobian();

Jm = J01;
Jm(:,8) = Jm(:,8) + Jm(:,9);
Jm(:,9) = [];

% Manuel defined things as row vectors
Cp_h = Cp_h.';
Cn_h = Cn_h.';

% Getting the vectors to compute the jacobian as if panda_hand frame were
% floating. This is done, gabiccini style

X = franka.get_forward_kinematics();
Cp_wrist = X(1:3,4,3); % the third element of the tensor is T to panda_hand
Cp_wl = Cp_h(:,1) - Cp_wrist; % left tip to wrist
Cp_wr = Cp_h(:,2) - Cp_wrist; % left tip to wrist

% Jacobian to left contact
Jleft1 = [1, 0, 0, 0, 0, 0].'; % col of first prismatic joint
Jleft2 = [0, 1, 0, 0, 0, 0].'; % col of second prismatic joint
Jleft3 = [0, 0, 1, 0, 0, 0].'; % col of third prismatic joint
Jleft4 = [cross([1 0 0].',Cp_wl); [1 0 0].']; % col of first rev joint
Jleft5 = [cross([0 1 0].',Cp_wl); [0 1 0].']; % col of second rev joint
Jleft6 = [cross([0 0 1].',Cp_wl); [0 0 1].']; % col of third rev joint

% Jacobian to right contact
Jright1 = [1, 0, 0, 0, 0, 0].'; % col of first prismatic joint
Jright2 = [0, 1, 0, 0, 0, 0].'; % col of second prismatic joint
Jright3 = [0, 0, 1, 0, 0, 0].'; % col of third prismatic joint
Jright4 = [cross([1 0 0].',Cp_wr); [1 0 0].']; % col of first rev joint
Jright5 = [cross([0 1 0].',Cp_wr); [0 1 0].']; % col of second rev joint
Jright6 = [cross([0 0 1].',Cp_wr); [0 0 1].']; % col of third rev joint

Jleft = [Jleft1, Jleft2, Jleft3, Jleft4, Jleft5, Jleft6];
Jright = [Jright1, Jright2, Jright3, Jright4, Jright5, Jright6];

% Putting together positions and orientations
Jpos = [Jleft(1:3,:); Jright(1:3,:)];
Jor = [Jleft(4:6,:); Jright(4:6,:)];

% Getting the H matrix
H = build_h(0,size(Cp_h.',1),0,Cn_h.'); % soft finger

% Selecting J's rows using H
HJf = H*[Jpos; Jor];

% Substituting in manipulator jacobian to get Jf
Jf = J01;
Jf(:,7) = [];
Jf(1:8,1:6) = HJf;

end

