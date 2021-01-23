function handle_tot = plot_contacts(Cp, Cn, RGB, scale)
if isempty(Cp)
    return;
end

if ~exist('RGB','var')
    RGB = [0 1 0];
end
if ~exist('scale','var')
    scale = 1;
end

handle_tot = {};

handle_tot{1} = plot3(Cp(:,1), Cp(:,2), Cp(:,3), 'r*');

% for j=1:size(Cp,1)
%     Cpn(j,:) = Cp(j,:) - Cn(j,:);
% end
Cpn = Cp - scale*Cn;
handle_tot{2} = quiver3(Cpn(:,1), Cpn(:,2), Cpn(:,3), scale*Cn(:,1), scale*Cn(:,2), scale*Cn(:,3), 0, ...
    'linewidth', 3.0, 'Color', RGB);
end