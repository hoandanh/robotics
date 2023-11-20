% -------------------------------------------------------------------------
% File : FinslerDynamicSwarmWindow.m
%
% Discription : Random Mobile Robots co-planning with Finsler Dynamic 
%               Window targeting Random Goal  
%
% Environment : Matlab R2016
% Authors : Atsushi Sakai(2014), Gao Li & Steed Huang(2017)
% Copyright (c) : 2014/2017
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------
function [] = MyDynamicWindowApproachObstacle()
close all;
clear all;
disp('Finsler Dynamic Window Approach swarm program start!')
global dt; 
dt = 0.2; % time [s]
global cola % collaboration component weight [0 to 1]
% Randomness of objects (robots, obstacles and goals)
delta = 0.72;
% Check if collaboration competition succeed
win = 3;
% Check if every one arrived
arrived = [0 0 0 0 0];
% Robot initial position
sites = [
1+delta*rand() 6+delta*rand()
0.5+delta*rand() 5+delta*rand()
delta*rand() 3+delta*rand()
0.5+delta*rand() 1+delta*rand()
1+delta*rand() delta*rand()
    ];    
% Convert to robot polar position to optimize matching with goal
[theata, rho] = cart2pol(sites(:,1),sites(:,2));
sitesP = sort([theata rho]);
[xx, yy] = pol2cart(sitesP(:,1),sitesP(:,2));
sitesR = [xx, yy];
% Goal initial position
sites = [
9+delta*rand() 6+delta*rand()
8+delta*rand() 5+delta*rand()
7+delta*rand() 4+delta*rand()
8+delta*rand() 3+delta*rand()
9+delta*rand() 2+delta*rand()
    ];    
% Convert to goal polar position to optimize matching with robot
[theata, rho] = cart2pol(sites(:,1),sites(:,2));
% When number of robot goes up£¬use Distributed Brain Storm Optimization
% for now we simply use this sorting algorithm
sitesP = sort([theata rho]);
[xx, yy] = pol2cart(sitesP(:,1),sitesP(:,2));
sitesG = [xx, yy];
% Calculate the azimuth for optimized initial moving direction
az1 = atan((sitesG(1,2)-sitesR(1,2))/(sitesG(1,1)-sitesR(1,1)));
az2 = atan((sitesG(2,2)-sitesR(2,2))/(sitesG(2,1)-sitesR(2,1)));
az3 = atan((sitesG(3,2)-sitesR(3,2))/(sitesG(3,1)-sitesR(3,1)));
az4 = atan((sitesG(4,2)-sitesR(4,2))/(sitesG(4,1)-sitesR(4,1)));
az5 = atan((sitesG(5,2)-sitesR(5,2))/(sitesG(5,1)-sitesR(5,1)));
goal1 = [sitesG(1,1) sitesG(1,2)];   % goal position [x(m),y(m)]
goal2 = [sitesG(2,1) sitesG(2,2)];   % goal position [x(m),y(m)]
goal3 = [sitesG(3,1) sitesG(3,2)];   % goal position [x(m),y(m)]
goal4 = [sitesG(4,1) sitesG(4,2)];   % goal position [x(m),y(m)]
goal5 = [sitesG(5,1) sitesG(5,2)];   % goal position [x(m),y(m)]
% Initial value for the collaboration
  cola = sqrt(cov((goal1+goal2+goal3+goal4+goal5)/5))/mean((goal1+goal2+goal3+goal4+goal5)/5)
% Number of robots
K = 5;
% This is Robot parameter set
x1 = [sitesR(1,1) sitesR(1,2) az1 0 0]';% robot initial status[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
x2 = [sitesR(2,1) sitesR(2,2) az2 0 0]';% robot initial status[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
x3 = [sitesR(3,1) sitesR(3,2) az3 0 0]';% robot initial status[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
x4 = [sitesR(4,1) sitesR(4,2) az4 0 0]';% robot initial status[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
x5 = [sitesR(5,1) sitesR(5,2) az5 0 0]';% robot initial status[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
% Obstacle position list [x(m) y(m)]
obstacle = [
4+delta*rand() 6+delta*rand()
3+delta*rand() 5+delta*rand()
4+delta*rand() 4+delta*rand()
3+delta*rand() 3+delta*rand()
4+delta*rand() 2+delta*rand()
];
% Radius for obstacle conflicting safe distance
obstacleR = 0.25;
% Robot motion model
% max speed[m/s],max turning speed[rad/s],acceleration[m/ss],turning acc speed[rad/ss],
% speed granularity[m/s],turning granularity[rad/s]]
Kinematic = [1.0,toRadian(30.0),0.2,toRadian(50.0),0.01,toRadian(1)];
% Evaluation function [heading,dist,velocity,predictDT] weights and predicted time [0.05,0.2,0.1,3.0]
evalParam = [0.05,0.07,0.1,3.0];
% Field [xmin xmax ymin ymax]
area = [-1 11 -1 8];
% Simulation result
result.x1 = [];
result.x2 = [];
result.x3 = [];
result.x4 = [];
result.x5 = [];
% Start the time
tic;
% Main loop
for i = 1:1000
    % Update the goal position
    goal1 = goal1*(1-sin(i*rand()/10)*exp(-i/10)/100);
    goal2 = goal2*(1-sin(i*rand()/10)*exp(-i/10)/100);
    goal3 = goal3*(1-sin(i*rand()/10)*exp(-i/10)/100);
    goal4 = goal4*(1-sin(i*rand()/10)*exp(-i/10)/100);
    goal5 = goal5*(1-sin(i*rand()/10)*exp(-i/10)/100);
    % Update the obstacle
    obstacle = obstacle*(1+rand()*sin(i)*exp(-i/10)/10);
    
    % Robot 1 head to goal directly
      if norm(x1(1:2)-goal1')>0.2
        % Update obstacle robot plus its mirror ghost
        obLenght = length(obstacle(:,1));
        obstacle(obLenght+1, :) = [x2(1) x2(2)]+[obstacleR obstacleR];
        obstacle(obLenght+2, :) = [x2(1)-2*x1(1) x2(2)-2*x1(2)]+[obstacleR obstacleR];
        % DWA inputs
        [u1,traj1] = DynamicWindowApproach(x1,Kinematic,goal1,evalParam,obstacle,obstacleR);
        x1 = f(x1,u1);% robot move to next step
        % Save the result and clean the memory 
        result.x1 = [result.x1; x1'];
        obstacle(obLenght+2, :) = [];
        obstacle(obLenght+1, :) = [];
      else
        arrived(1) = 1;
      end
    % Robot 2 competes with robot 3
      if (norm(x2(1:2)-goal2')>0.2 && win~=2) || (norm(x2(1:2)-goal3')>0.2 && win==2)
        obLenght = length(obstacle(:,1));
        obstacle(obLenght+1, :) = [x1(1) x1(2)]+[obstacleR obstacleR];
        obstacle(obLenght+2, :) = [x3(1) x3(2)]+[obstacleR obstacleR];
        % Create virtual goal
        w2 = CalcDistGoal(goal2,[x2(1) x2(2)]);
        w3 = CalcDistGoal(goal3,[x2(1) x2(2)]);
        goal2v = (goal2*w3+goal3*w2)/(w2+w3); 
        % Check if Robot 2 won the competition
          if w3 < w2
             win = 2;
          end
        % DWA inputs
        [u2,traj2] = DynamicWindowApproach(x2,Kinematic,goal2v,evalParam,obstacle,obstacleR);
        x2 = f(x2,u2);% robot move to next step
        % Save the result and clean the memory
        result.x2 = [result.x2; x2'];
        obstacle(obLenght+2, :) = [];
        obstacle(obLenght+1, :) = [];
      else
        arrived(2) = 1;
      end
      
    % Robot 3 collaborates with robot 2 and 4 
      if (norm(x3(1:2)-goal3')>0.2 && win==3) || (norm(x3(1:2)-goal2')>0.2 && win==2) || (norm(x3(1:2)-goal4')>0.2 && win==4) 
        obLenght = length(obstacle(:,1));
        obstacle(obLenght+1, :) = [x2(1) x2(2)]+[obstacleR obstacleR];
        obstacle(obLenght+2, :) = [x4(1) x4(2)]+[obstacleR obstacleR];
        % Create virtual goal
        w2 = CalcDistGoal([x3(1) x3(2)],[x2(1) x2(2)]);
        w4 = CalcDistGoal([x3(1) x3(2)],[x4(1) x4(2)]);
        % Add a little bit heading room
        goal3g = ([x2(1) x2(2)]*w4+[x4(1) x4(2)]*w2)/(w2+w4)+[obstacleR obstacleR]; 
        % Check if goal is close enough that collaboration turns into
        % Competition then, with 10 time weight
        if CalcDistGoal([x3(1) x3(2)],goal3)<10*CalcDistGoal([x3(1) x3(2)],goal3g)
            % When Robot 3 loss to Robot 4 or Robot 2, swap the goals
            if win == 4
            goal3g = goal4;
            elseif win == 2
            goal3g = goal2;
            else
            goal3g = goal3;
            end
        end
        % DWA inputs
        [u3,traj3] = DynamicWindowApproach(x3,Kinematic,goal3g,evalParam,obstacle,obstacleR);
        x3 = f(x3,u3);% robot move to next step
        % Save the result and clean the memory
        result.x3 = [result.x3; x3'];
        obstacle(obLenght+2, :) = [];
        obstacle(obLenght+1, :) = [];
      else
        arrived(3) = 1;
      end
      % Check if Robot 3 won over
        if norm(x3(1:2)-goal2')<0.25
        win = 2;
        elseif norm(x3(1:2)-goal4')<0.25
        win = 4;
        end   
    % Robot 4 competes with robot 3 
      if (norm(x4(1:2)-goal4')>0.2 && win~=4) || (norm(x4(1:2)-goal3')>0.2 && win==4)
        obLenght = length(obstacle(:,1));
        obstacle(obLenght+1, :) = [x3(1) x3(2)]+[obstacleR obstacleR];
        obstacle(obLenght+2, :) = [x5(1) x5(2)]+[obstacleR obstacleR];
        % Create virtual goal
        w4 = CalcDistGoal(goal4,[x4(1) x4(2)]);
        w3 = CalcDistGoal(goal3,[x4(1) x4(2)]);
        goal4v = (goal4*w3+goal3*w4)/(w4+w3); 
        % Check if Robot 4 won the competition
          if w3 < w4
             win = 4;
          end
        % DWA inputs
        [u4,traj4] = DynamicWindowApproach(x4,Kinematic,goal4v,evalParam,obstacle,obstacleR);
        x4 = f(x4,u4);% robot move to next step
        % Save the result and clean the memory
        result.x4 = [result.x4; x4'];
        obstacle(obLenght+2, :) = [];
        obstacle(obLenght+1, :) = [];
      else
        arrived(4) = 1;
      end
    % Robot 5 heading to the goal directly
      if norm(x5(1:2)-goal5')>0.2
        % Update obstacle robot plus its mirror ghost
        obLenght = length(obstacle(:,1));
        obstacle(obLenght+1, :) = [x4(1) x4(2)]+[obstacleR obstacleR];
        obstacle(obLenght+2, :) = [x4(1)-2*x5(1) x4(2)-2*x5(2)]+[obstacleR obstacleR];
        % DWA inputs
        [u5,traj5] = DynamicWindowApproach(x5,Kinematic,goal5,evalParam,obstacle,obstacleR);
        x5 = f(x5,u5);% robot move to next step
        % Save the result and clean the memory
        result.x5 = [result.x5; x5'];
        obstacle(obLenght+2, :) = [];
        obstacle(obLenght+1, :) = [];
      else
        arrived(5) = 1;
      end
    % Define swarm collaboration terms
        V = (u1(1)+u2(1)+u3(1)+u4(1)+u5(1))/5;
        W = (u1(2)+u2(2)+u3(2)+u4(2)+u5(2))/5;
    % Calculate the collaboration level by hypo correlation coefficent in
    % C*-Algebra Finsler manifold
        colaNomi = (u1(1)-V)^(3/4)*(u1(2)-W)^(3/4)+(u2(1)-V)^(3/4)*(u2(2)-W)^(3/4)+(u3(1)-V)^(3/4)*(u3(2)-W)^(3/2)+(u4(1)-V)^(3/4)*(u4(2)-W)^(3/2)+(u5(1)-V)^(3/4)*(u5(2)-W)^(3/2);
        colaDeno = ((u1(1)-V)^(3/2)+(u2(1)-V)^(3/2)+(u3(1)-V)^(3/2)+(u4(1)-V)^(3/2)+(u5(1)-V)^(3/2))*((u1(2)-W)^(3/2)+(u2(2)-W)^(3/2)+(u3(2)-W)^(3/2)+(u4(2)-W)^(3/2)+(u5(2)-W)^(3/2));
        cola = abs(colaNomi^(4/3)/max(colaDeno^(2/3), 1));
    % Check if everybody arrived
    if arrived(1)==1 && arrived(2)==1 && arrived(3)==1 && arrived(4)==1 && arrived(5)==1 
        disp('Arrive Goal!');
        break;
    end
    
    %====Animation====
    hold off;
    ArrowLength = 0.5;
    
    plot(goal1(1),goal1(2),'h','MarkerSize',12);hold on;plot(goal2(1),goal2(2),'h','MarkerSize',12);hold on;
    plot(goal3(1),goal3(2),'h','MarkerSize',12);hold on;plot(goal4(1),goal4(2),'h','MarkerSize',12);hold on;
    plot(goal5(1),goal5(2),'h','MarkerSize',12);hold on;
    plot(obstacle(:,1),obstacle(:,2),'s','MarkerSize',12);hold on;
    axis(area);
    grid on;
    title ('Finsler Dynamic Window Swarm Algorithm')  
    xlabel('X: Star is the Goal');
    ylabel('Y: Circle is the Robot');
    text(0,0,'Square is the Obstacle');
    % Robot1
    quiver(x1(1),x1(2),ArrowLength*cos(x1(3)),ArrowLength*sin(x1(3)),'ok');hold on;
    plot(result.x1(:,1),result.x1(:,2));hold on;
    % Search for potential trace
    if ~isempty(traj1)
        for it = 1:length(traj1(:,1))/5
            ind = 1+(it-1)*5;
            plot(traj1(ind,:),traj1(ind+1,:),'-g');hold on;
        end
    end
    % Robot2
    quiver(x2(1),x2(2),ArrowLength*cos(x2(3)),ArrowLength*sin(x2(3)),'ok');hold on;
    plot(result.x2(:,1),result.x2(:,2));hold on;
    % Search for potential trace
    if ~isempty(traj2)
        for it = 1:length(traj2(:,1))/5
            ind = 1+(it-1)*5;
            plot(traj2(ind,:),traj2(ind+1,:),'-g');hold on;
        end
    end
    % Robot3
    quiver(x3(1),x3(2),ArrowLength*cos(x3(3)),ArrowLength*sin(x3(3)),'ok');hold on;
    plot(result.x3(:,1),result.x3(:,2));hold on;
    % Search for potential trace
    if ~isempty(traj3)
        for it = 1:length(traj3(:,1))/5
            ind = 1+(it-1)*5;
            plot(traj3(ind,:),traj3(ind+1,:),'-g');hold on;
        end
    end
    % Robot4
    quiver(x4(1),x4(2),ArrowLength*cos(x4(3)),ArrowLength*sin(x4(3)),'ok');hold on;
    plot(result.x4(:,1),result.x4(:,2));hold on;
    % Search for potential trace
    if ~isempty(traj4)
        for it = 1:length(traj4(:,1))/5
            ind = 1+(it-1)*5;
            plot(traj4(ind,:),traj4(ind+1,:),'-g');hold on;
        end
    end
    % Robot5
    quiver(x5(1),x5(2),ArrowLength*cos(x5(3)),ArrowLength*sin(x5(3)),'ok');hold on;
    plot(result.x5(:,1),result.x5(:,2));hold on;
    % Search for potential trace
    if ~isempty(traj5)
        for it = 1:length(traj5(:,1))/5
            ind = 1+(it-1)*5;
            plot(traj5(ind,:),traj5(ind+1,:),'-g');hold on;
        end
    end
    % Draw together
    drawnow;
end
% Stop the time
toc
% Dynamic Window [vmin,vmax,wmin,wmax]
% Find min max move speed and turn speed
function [u,trajDB] = DynamicWindowApproach(x,model,goal,evalParam,ob,R)
global cola 
Vr = CalcDynamicWindow(x,model);  
% Evaluation for DB[vt ot heading dist vel]
[evalDB,trajDB] = Evaluation(x,Vr,goal,ob,R,model,evalParam);
if isempty(evalDB)
    disp('no path to goal!!');
    u = [0;0];return;
end
% Normalize evaluation function
evalDB = NormalizeEval(evalDB);
% Final evaluation
feval = [];
for id = 1:length(evalDB(:,1))
    feval = [feval;evalParam(1:3)*evalDB(id,3:5)' + cola];
    % test1 = feval
end
% Put evaluation result into final decision
evalDB = [evalDB feval];  
% Find index for the best choice
[maxv,ind] = max(feval);  
% line speed¡¢angle speed
u = evalDB(ind,1:2)';     
% Status and trace estimation
function [evalDB,trajDB] = Evaluation(x,Vr,goal,ob,R,model,evalParam)
evalDB = [];  
trajDB = [];  % trace window
for vt = Vr(1):model(5):Vr(2)
    for ot = Vr(3):model(6):Vr(4)
        % trace inference from xt, predicted robot trace
        [xt,traj] = GenerateTrajectory(x,vt,ot,evalParam(4),model); 
        % individual goal evaluation
        heading = CalcHeadingEval(xt,goal);
        dist = CalcDistEval(xt,ob,R);
        vel = abs(vt);
        % breaking distance
        stopDist = CalcBreakingDist(vel,model);
        if dist>stopDist 
            evalDB = [evalDB;[vt ot heading dist vel]];
            trajDB = [trajDB;traj];
        end
    end
end
% Normalizing evaluation
function EvalDB = NormalizeEval(EvalDB)
if sum(EvalDB(:,3))~=0
    EvalDB(:,3) = EvalDB(:,3)/sum(EvalDB(:,3));
end
if sum(EvalDB(:,4))~=0
    EvalDB(:,4) = EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5) = EvalDB(:,5)/sum(EvalDB(:,5));
end
% Trace generating
% evaldt£ºforward time; vt: current speed;
function [x,traj] = GenerateTrajectory(x,vt,ot,evaldt,model)
global dt;
time = 0;
u = [vt;ot];
traj = x;
while time<=evaldt
    % Time update
    time = time+dt;
    x = f(x,u);
    % Move update
    traj = [traj x];
end
% Dynamic model for breaking distance, ignore turning
function stopDist = CalcBreakingDist(vel,model)
global dt;
stopDist = 0;
while vel>0
    % Calculate stop distance
    stopDist = stopDist+vel*dt;
    vel = vel-model(3)*dt;
end
% Obstacle distance evaluation
function dist = CalcDistEval(x,ob,R)
dist = 100;
for io=1:length(ob(:,1))
    % disttmp=norm(ob(io,:)-x(1:2)')-R; 
    % Replacing Euclidean with Finsler!
    disEucNor = norm(ob(io,:)-x(1:2)',2)-10*R;
    % Go through Sigmoid function to make a Lobachevskii-Riemann measure
    dimR = 1.5+1/(1+exp(-2*disEucNor));
    % Infinite Finsler Manifold
    disttmp = norm(ob(io,:)-x(1:2)',dimR)-R;
    % Finding the minimum obstacle distance
    if disttmp > 0 && dist>disttmp     
        dist = disttmp;
    end
end
% Introduce the ghost obstacle to avoid over-optimistic
if dist>=2*R
    dist = 2*R;
end
% Goal distance evaluation
function dist = CalcDistGoal(x,go)
dist = 10;
    % Replacing Euclidean with Finsler!
    disEucNo = norm(x-go,2);
    % Go through Sigmoid function to make a Lobachevskii-Riemann measure
    dimRg = 1.5+1/(1+exp(-2*disEucNo));
    % Infinite Finsler Manifold
    dist = norm(x-go,dimRg)^2.5;
% Heading evaluation
function heading = CalcHeadingEval(x,goal)
% Robot direction
theta = toDegree(x(3));
% Goal direction
goalTheta = toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));
if goalTheta>theta
    targetTheta = goalTheta-theta;% [deg]
else
    targetTheta = theta-goalTheta;% [deg]
end
heading = 180-targetTheta;
% Make dynamic window
function Vr = CalcDynamicWindow(x,model)
global dt;
% Speed range
Vs = [0 model(1) -model(2) model(2)];
% Based on current speed and speed limit forming the dynamic window
Vd = [x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
% Final; Dynamic Window
Vtmp = [Vs;Vd];
Vr = [max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
% Motion Model equation
% u = [vt; wt];current speed¡¢angle speed
% X = X + v*cos(¦È)*dt
% Y = Y + v*sin(¦È)*dt
% ¦È = ¦È + w*dt
% v = v
% w = w
function x = f(x, u)
global dt;
F = [1 0 0 0 0
    0 1 0 0 0
    0 0 1 0 0
    0 0 0 0 0
    0 0 0 0 0];
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];
x = F*x+B*u;
% Degree to radian
function radian = toRadian(degree)
radian = degree/180*pi;
% Radian to degree
function degree = toDegree(radian)
degree = radian/pi*180;