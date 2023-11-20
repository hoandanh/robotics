close all; clear; clc;

%% Demo algorithm
show_animation = 0;

config_init = Config();
disp('Spherical-Particle Swarm Optimization sample program start!!');
BestPosition = SPSO(show_animation, config_init);

global_traj = [BestPosition.x; BestPosition.y]';
global_traj = [config_init.start; global_traj; config_init.end];
N = size(global_traj, 1);

disp('Dynamic Window Approach sample program start!!');
local_traj = [];
for i=1:N-1
    config = Config(global_traj(i,:), global_traj(i+1,:));
    result = DWA(show_animation, config);
    local_traj = [local_traj; result.x];
end

%% Plot the result
figure;
hold on;
plot(config_init.end(1),config_init.end(2),'*r');
plot(config_init.threats(:,1),config_init.threats(:,2),'*k');
plot(global_traj(:,1),global_traj(:,2),'r');
plot(local_traj(:,1),local_traj(:,2),'-b');
xlabel('x(m)');
ylabel('y(m)');
grid on;
    