clear; clc;

n = 5001;

pD0 = [0,10];
pDt = [0,70];
ptheta = [0,180];

D0 = pD0(1) + rand(1,n)*(pD0(2) - pD0(1));
Dt = pDt(1) + rand(1,n)*(pDt(2) - pDt(1));
theta = ptheta(1) + rand(1,n)*(ptheta(2) - ptheta(1));

input = [D0;Dt;theta];
fis = readfis('dwa_fuzzy');

alpha = []; beta = []; gamma = [];

for i=1:length(D0)
    weight = evalfis(fis,input(:,i));
    alpha = [alpha,weight(1)];
    beta  = [ beta,weight(2)];
    gamma = [gamma,weight(3)];
end

alpha = [input;alpha]';
beta  = [input; beta]';
gamma = [input;gamma]';
