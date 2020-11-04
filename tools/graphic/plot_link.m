function handl = plot_link(T_init, T_final, r, is_end_effector, RGB, axis)

if ~exist('axis', 'var')
    axis = 'n';
end

l_m = 0;
if is_end_effector
    l_m = r;
end

p1 = T_init(1:3, 4);
p2 = T_final(1:3, 4);
p_delta = p2 - p1;
R = zeros(3, 3);
z = p_delta / norm(p_delta);
R(:, 3) = z;
x_rand = rand(3, 1) - 0.5;
x_tmp = (eye(3) - z*z.') * x_rand;
x = x_tmp / norm(x_tmp); % random vector but orthogonal to z
R(:, 1) = x;
y = cross(z, x);
R(:, 2) = y;

theta = linspace(0, 2*pi, 20);

X1 = p1 + R * [r * cos(theta);
    r * sin(theta); zeros(size(theta)) + l_m];
X2 = p1 + p_delta + R * [r * cos(theta);
    r * sin(theta); zeros(size(theta)) - l_m];

handl(1) = surf([X1(3,:); X2(3,:)], [X1(1,:); X2(1,:)], [X1(2,:); X2(2,:)], ...
    'edgecolor','none', 'FaceColor', RGB);

[sx, sy, sz] = sphere;
hold on
handl(2) = surf(sz.*r + T_init(3,4), sx.*r + T_init(1,4), sy.*r + T_init(2,4), ...
    'edgecolor','none', 'FaceColor', RGB);

if ~is_end_effector
    handl(3) = surf(sz.*r + T_final(3,4), sx.*r + T_final(1,4), ...
        sy.*r + T_final(2,4), 'edgecolor','none', 'FaceColor', RGB);
else
    X1 = X2;
    X2 = p1 + p_delta + R*[0*cos(theta);
        0*sin(theta);
        zeros(size(theta))];
    handl(3) = surf([X1(3,:); X2(3,:)], [X1(1,:); X2(1,:)], [X1(2,:); X2(2,:)], ...
        'edgecolor','none', 'FaceColor', RGB);
end

if axis == 'n'
    handl(4) = 0;
    return;
end

scale = 0.25;
if axis == 'z'
    m_axis = T_init(1:3,1:3)*[0 0 1].';
    p = T_init(1:3,4) + T_init(1:3,1:3)*[0 0 -scale/2].';
elseif axis == 'y'
    m_axis = T_init(1:3,1:3)*[0 1 0].';
    p = T_init(1:3,4) + T_init(1:3,1:3)*[0 -scale/2 0].';
else
    m_axis = T_init(1:3,1:3)*[1 0 0].';
    p = T_init(1:3,4) + T_init(1:3,1:3)*[-scale/2 0 0].';
end

handl(4) = quiver3(p(3), p(1),p(2), m_axis(3), m_axis(1), m_axis(2), scale, ...
    'linewidth', 3.0, 'Color', [0 1 0]);
end