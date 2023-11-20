function [evalDB,trajDB,weights]=CalcControlAndTrajectory(x,dw,config)
    % A function that computes an evaluation value for each pass
    evalDB=[];
    trajDB=[];
    weights = [];
    
    for vt=dw(1):config.v_resolution:dw(2)
        for ot=dw(3):config.yaw_rate_resolution:dw(4)
            % Trajectory Estimation
            [xt,traj]=GenerateTrajectory(x,vt,ot,config);
            % Calculation of each merit function
            heading=CalcHeadingEval(xt,config); % theta
            dist=CalcDistEval(xt,config); % D0
            vel=abs(vt);
            toGoal=norm(xt(1:2)-config.end); % Dt
            
            weight = estimateW(dist,toGoal,abs(heading),config);
            weight = weight/norm(weight,1);
            
            stopDist=CalcBreakingDist(vel,config);
            if dist > stopDist
                evalDB=[evalDB;[vt ot heading dist vel]];
                trajDB=[trajDB;traj];
                weights=[weights;weight];
            end
        end
    end
end