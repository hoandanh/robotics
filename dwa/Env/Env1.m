clear; close all; clc;

x = [];y=[];

obsy = [linspace(30,40,51);
        linspace(20,30,51);
        linspace(10,20,51)];

obsx = [linspace(10,30,51);
        linspace(0,10,51)];

y = [y,obsy(1,:),obsy(2,:),obsy(3,:)];
x = [x,10*ones(1,length(obsy(1,:))),40*ones(1,length(obsy(2,:))),30*ones(1,length(obsy(3,:)))];

x = [x,obsx(1,:),obsx(2,:)];
y = [y,20*ones(1,length(obsx(1,:))),10*ones(1,length(obsx(2,:)))];

[x,y] = configObs(x,y,[35,35,45,45]);
R = 0.15*ones(1,length(x));

plotEnv([0,0],[50,50],x,y);

save("Env1.mat","x","y","R");
