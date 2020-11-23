function handle = plot_contacts(Cp, Cn, RGB)
if isempty(Cp)
    return;
end

if ~exist('RGB','var')
    RGB = [0 1 0];
end

handle(1) = plot3(Cp(:,3), Cp(:,1), Cp(:,2), 'r*');
Cpn = Cp - Cn;
handle(2) = quiver3(Cpn(:,3), Cpn(:,1), Cpn(:,2), Cn(:,3), Cn(:,1), ...
                    Cn(:,2), 0, 'linewidth', 3.0, 'Color', RGB);
end