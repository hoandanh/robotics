function [evalDB,trajDB]=CalcControlAndTrajectory(x,dw,config)
    % A function that computes an evaluation value for each pass
    evalDB=[];
    trajDB=[];
    
    for vt=dw(1):config.v_resolution:dw(2)
        for ot=dw(3):config.yaw_rate_resolution:dw(4)
            % Trajectory Estimation
            [xt,traj]=GenerateTrajectory(x,vt,ot,config);
            % Calculation of each merit function
            heading=CalcHeadingEval(xt,config); % theta
            dist=CalcDistEval(xt,config); % D0
            vel=abs(vt);
            
            stopDist=CalcBreakingDist(vel,config);
            if dist > stopDist
                evalDB=[evalDB;[vt ot heading dist vel]];
                trajDB=[trajDB;traj];
            end
        end
    end
end