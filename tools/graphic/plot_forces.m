function plot_forces(Cp, Cf, RGB)
if isempty(Cp)
    return;
end

if ~exist('RGB','var')
    RGB = [0 1 0];
end

plot3(Cp(:,3), Cp(:,1), Cp(:,2), 'r*')

% for j=1:size(Cp,1)
%     Cpn(j,:) = Cp(j,:) - Cn(j,:);
% end
Cpf = Cp + Cf;
quiver3(Cp(:,3), Cp(:,1), Cp(:,2), Cpf(:,3), Cpf(:,1), Cpf(:,2), 0, ...
    'linewidth', 3.0, 'Color', RGB)
end