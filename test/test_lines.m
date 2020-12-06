p1 = [1; 2];
p2 = [3; 1];

plot(p1(1), p1(2), 'r*');
hold on;
plot(p2(1), p2(2), 'c*');
for t = linspace(0, 0.9, 10)
    p = (1-t)*p1 + t*p2;
    plot(p(1), p(2), 'o');
end
