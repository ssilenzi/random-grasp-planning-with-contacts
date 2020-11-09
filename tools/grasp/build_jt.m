function Jt = build_jt(Cp, Org, Zax, Rel)

% function Jt = build_jt(Cp,Cn,Org,Zax,Rel,l):
%
% Builds the "Jacobian" matrix Jt for a configuration with n contact 
% point c_i. 
% Contact point coordinates in base frame are passed as rows of Cp (nx3);
% Origin and Z-axis vectors of joint frames are passed as 
% rows of Org (mx3) and Zax (mx3), resp;
% Matrix Rel (nxm) has
% Rel(i,j) = 0 if the i-th contact does not affect the j-th joint torque;
% Rel(i,j) = 1 if the j-th joint is rotoidal;
% Rel(i,j) = 2 if the j-th joint is prismatic.

[n, ~] = size(Cp);
[m, ~] = size(Org);
Zax = (normal(Zax.')).';

Jp = [];
for j = 1:m
    Jtrow = [];
    for i = 1:n
        if Rel(i,j) == 0
            D = [0, 0, 0];
        elseif Rel(i,j) == 1  
            CmO = Cp(i,:) - Org(j,:);
            CmOx = cross_p_matrix(CmO);
            D = -Zax(j,:) * CmOx;
            % the minus has been added to correct error
            % (confirmed by symbolic test file)
        elseif Rel(i,j) == 2
            D = Zax(j,:);
        end
        Jtrow = [Jtrow, D];
    end
    Jp = [    Jp;
           Jtrow];
end

Jo = [];
for j = 1:m
    Jtrow = [];
    for i = 1:n
        if Rel(i,j) == 0 || Rel(i,j) == 2
            D = [0, 0, 0];
        elseif Rel(i,j) == 1  
            D = Zax(j,:);
        end
        Jtrow = [Jtrow, D];
    end
    Jo = [    Jo;
           Jtrow];
end

Jt = [Jp, Jo];
end