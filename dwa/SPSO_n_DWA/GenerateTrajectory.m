function [x,traj]=GenerateTrajectory(x,vt,ot,config)
    % Function to create trajectory data
    dt = config.dt;
    time = 0;
    u = [vt;ot]; % Input value
    traj = x; % Trajectory data
    while time <= config.predict_time
        time = time + dt; %Simulation time update
        x = f(x,u,dt); % Movement model
        traj = [traj x];
    end
end