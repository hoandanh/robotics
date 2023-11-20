function [x,y] = configObs(x,y,obs)
    x_range = obs(1):0.2:obs(3);
    y_range = obs(2):0.2:obs(4);
    for i=1:length(x_range)
        x = [x,x_range(i)];
        y = [y,obs(2)];
    end
    for i=1:length(x_range)
        x = [x,x_range(i)];
        y = [y,obs(4)];
    end
    for i=1:length(y_range)
        y = [y,y_range(i)];
        x = [x,obs(1)];
    end
    for i=1:length(y_range)
        y = [y,y_range(i)];
        x = [x,obs(3)];
    end
end