function T_rand = rand_samp_T()
% Random sampling of an object configuration

t_range = 2; % needs to be given from outside later
ang_range = 2*pi;

% Random sampling position
t_rand = 2*t_range*(rand(3,1) - 0.5);

% Random sampling rpy and converting into rotation
r_rand = 2*ang_range*(rand - 0.5);
p_rand = 2*ang_range*(rand - 0.5);
y_rand = 2*ang_range*(rand - 0.5);
R_rand = eul2rotm([y_rand p_rand r_rand],'zyx');

T_rand = [R_rand, t_rand;
    zeros(1,3), 1];

end