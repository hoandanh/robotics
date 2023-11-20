clear; close all; clc;

x = []; y = [];

obs1 = [2,7,6,11];
[x,y] = configObs(x,y,obs1);

obs2 = [2,2,6,5];
[x,y] = configObs(x,y,obs2);

obs3 = [14,3,17,7];
[x,y] = configObs(x,y,obs3);

obs4 = [9,8,12,14]; % L
x_range = obs4(1):0.2:obs4(3);
y_range = obs4(2):0.2:obs4(4);
for i=1:length(x_range)
    x = [x,x_range(i)];
    y = [y,obs4(2)];
end
for i=1:length(y_range)
    y = [y,y_range(i)];
    x = [x,obs4(1)];
end

obs5 = [12,8,14,9]; % L
x_range = obs5(1):0.2:obs5(3);
y_range = obs5(2):0.2:obs5(4);
for i=1:length(x_range)
    x = [x,x_range(i)];
    y = [y,obs5(4)];
end
for i=1:length(y_range)
    y = [y,y_range(i)];
    x = [x,obs5(3)];
end

R = 0.15*ones(1,length(x));

x = [x,10];
y = [y,3];
R = [R,1.0];

plotEnv([0,0],[15,15],x,y);

save("Env2.mat","x","y","R");
