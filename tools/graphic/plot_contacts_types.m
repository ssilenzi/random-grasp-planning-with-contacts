function plot_contacts_types(Cp, Cn, c_types, mu, t, RGB)
plot3( Cp(:,1), Cp(:,2), Cp(:,3), 'r*')
% axis([-2 2 -2 2 -2 2])
if ~exist('RGB', 'var')
    RGB = rand(1,3);
end

Cpn = [];
Cnn = [];
for j=1:size(Cp,1)
    if c_types(j) == 1  
        Cnn = [Cnn; Cn(j,:)];
        Cpn =[Cpn; Cp(j,:) - Cn(j,:)];
    elseif c_types(j) == 3
        GG_s_i = build_g(Cp(j,:),1);
        H_s_i = build_h(0,0,1,Cn(j,:));
        c_e_p = H_s_i*GG_s_i.'*t;
        S_i = Cn(j,:).' - mu*c_e_p/norm(c_e_p);
        % to be generalized for any contact type
        % GesS = [GesS GG_s_i*S_i];
        Cnn = [Cnn; S_i.'/norm(S_i)];
        Cpn = [Cpn; Cp(j,:) - S_i.'/norm(S_i)];
    end
end
quiver3(Cpn(:,1), Cpn(:,2), Cpn(:,3),  Cnn(:,1), Cnn(:,2), Cnn(:,3), ...
    0, 'linewidth', 3.0, 'Color', RGB)
end