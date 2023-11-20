function [u,trajDB]=DynamicWindowApproach(x,config)
    % Function to calculate the input value by DWA

    % Create Dynamic Window[vmin,vmax,ωmin,ωmax]
    dw=CalcDynamicWindow(x,config);

    % Calculating the Merit Function
    [evalDB,trajDB,weights]=CalcControlAndTrajectory(x,dw,config);

    if isempty(evalDB)
        disp('no path to goal!!');
        u=[0;0];return;
    end

    % Normalization of each merit function
    evalDB=NormalizeEval(evalDB);
    
    % Final Rating Value Calculation
    feval=[];
    for id=1:length(evalDB(:,1))
        feval=[feval;weights(id,:)*evalDB(id,3:5)'];
    end
    evalDB=[evalDB feval];

    % Calculate the index of the input value with the highest evaluation value
    [~,ind]=max(feval);
    % Return the input value with the highest % rated value
    u=evalDB(ind,1:2)';
    if abs(u(1)) < config.robot_stuck_flag_cons &&...
            abs(x(4)) < config.robot_stuck_flag_cons
        u(2) = -config.max_delta_yaw_rate;
    end
end