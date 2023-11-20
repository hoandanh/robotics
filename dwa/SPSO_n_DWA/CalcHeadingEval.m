function heading=CalcHeadingEval(x,config)
    goal = config.end;
    % A function that computes the merit function of "heading"

    theta=toDegree(x(3));% Robot heading
    goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1))); % Goal direction

    if goalTheta>theta
        targetTheta=goalTheta-theta; % Azimuth difference to goal [deg]
    else
        targetTheta=theta-goalTheta; % Azimuth difference to goal [deg]
    end

    heading=180-targetTheta;
end