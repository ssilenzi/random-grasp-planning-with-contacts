% Trying to create a symbolic version of FK and Jacobian

clear all;
clc;

syms x y z yaw pit rol q1 q2 l1 l2 l3 l4 real;

% Forward Kinematics

T= zeros(4,4,10);
% csys to hand base
T1 = transl(x,0,0);
T2 = T1 * transl(0,y,0);
T3 = T2 * transl(0,0,z);
T4 = T3 * trotz(yaw);
T5 = T4 * troty(pit);
T6 = T5 * trotx(rol);
% csys to end effector
T7 = T6 * transl(0, l1, 0); % p1
T8 = T7 * trotx(q1) * transl(0, l2, 0); % p2
T9 = T8 * trotx(q2) * transl(0, l3, 0); % p3, Cp1
T10 = T6 * transl(0, 0, l4); % p4, Cp2
% output fk of 9, 10, 6
T_all = T;
X1 = T9; pos1 = X1(1:3,4); 
R1 = X1(1:3,1:3); or1 = rotm2eul(R1,'zyx'); % not working
X2 = T10; pos2 = X2(1:3,4);
R2 = X2(1:3,1:3); or2 = rotm2eul(R2,'zyx'); % not working
Xw = T6; posw = Xw(1:3,4);
Rw = Xw(1:3,1:3); orw = rotm2eul(Rw,'zyx'); % not working

% Jacobian computiation
% J1 = simplify(jacobian(pos1, [x; y; z; yaw; pit; rol; q1; q2]));
% J2 = simplify(jacobian(pos2, [x; y; z; yaw; pit; rol; q1; q2]));
% Jw = simplify(jacobian(posw, [x; y; z; yaw; pit; rol; q1; q2]));
Jo1 = simplify(jacobian(or1, [x; y; z; yaw; pit; rol; q1; q2])); % not working
Jo2 = simplify(jacobian(or2, [x; y; z; yaw; pit; rol; q1; q2])); % not working
Jow = simplify(jacobian(orw, [x; y; z; yaw; pit; rol; q1; q2])); % not working


