function plot_points(Cp, RGB)
if isempty(Cp)
    return;
end

if ~exist('RGB','var')
    RGB = [0 1 0];
end

plot3(Cp(:,1), Cp(:,2), Cp(:,3), 'r*', 'linewidth', 3.0)

end