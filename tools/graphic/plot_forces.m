function plot_forces(Cp, Cf, RGB, scale)
if isempty(Cp)
    return;
end

if ~exist('RGB','var')
    RGB = [0 0.5 0.5];
end

if ~exist('scale','var')
    scale = 1;
end

plot3(Cp(:,3), Cp(:,1), Cp(:,2), 'r*')

quiver3(Cp(:,3), Cp(:,1), Cp(:,2), ...
    scale.*Cf(:,3), scale.*Cf(:,1), scale.*Cf(:,2), ...
    0, 'linewidth', 3.0, 'Color', RGB)
end