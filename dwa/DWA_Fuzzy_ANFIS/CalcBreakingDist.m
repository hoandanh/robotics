function stopDist=CalcBreakingDist(vel,config)
    % A function that calculates the braking distance from the current speed
    % according to the dynamics model
    dt = config.dt;
    stopDist=0;
    while vel>0
        stopDist=stopDist+vel*dt; % Braking distance calculation
        vel=vel-config.max_accel*dt; % Best Principle
    end
end