function plot_points_color(Cp, RGB)
if isempty(Cp)
    return;
end

if ~exist('RGB','var')
    RGB = [0 1 0];
end

col_str = 'rs';
if RGB(1)
    col_str = 'rs';
elseif RGB(2)
    col_str = 'gs';
elseif RGB(3)
    col_str = 'bs';
end

plot3(Cp(:,1), Cp(:,2), Cp(:,3), col_str, 'linewidth', 3.0)

end