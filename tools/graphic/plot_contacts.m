function plot_contacts(Cp, Cn, RGB)
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
Cpn = Cp - Cn;
quiver3(Cpn(:,3), Cpn(:,1), Cpn(:,2), Cn(:,3), Cn(:,1), Cn(:,2), 0, ...
    'linewidth', 3.0, 'Color', RGB)
end