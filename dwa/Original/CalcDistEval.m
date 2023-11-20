function dist=CalcDistEval(x,config)
    % Function to calculate the distance evaluation value with obstacles

    ob = config.threats(:,1:2);
    R = config.threats(:,3);
    
    dist=2;
    for io = 1:length(ob(:,1))
        disttmp = norm(ob(io,:) - x(1:2)') - R(io) - config.robot_radius; % Calculate norm error between path position and obstacle
        if dist > disttmp % Find the minimum
            dist = disttmp;
        end
    end
end