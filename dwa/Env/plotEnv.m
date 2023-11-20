function plotEnv(s,g,x,y)

    max_x = max(x) + 2;
    max_y = max(y) + 2;

    area = [-2 max_x -2 max_y]; axis(area); grid on; hold on;
    plot(s(1),s(2),'*b');hold on;
    plot(g(1),g(2),'*r');hold on;
    plot(x,y,'.k');
end