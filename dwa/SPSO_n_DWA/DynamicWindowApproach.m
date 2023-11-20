function [u,trajDB]=DynamicWindowApproach(x,config)
    % Function to calculate the input value by DWA

    % Create Dynamic Window[vmin,vmax,ωmin,ωmax]
    dw=CalcDynamicWindow(x,config);

    % Calculating the Merit Function
    [u,trajDB]=CalcControlAndTrajectory(x,dw,config);

    if isempty(u)
        disp('no path to goal!!');
        u=[0;0];return;
    end

    % Normalization of each merit function
    u=NormalizeEval(u);

    % Final Rating Value Calculation
    feval=[];
    for id=1:length(u(:,1))
        feval=[feval;[config.alpha config.gamma config.beta]*u(id,3:5)'];
    end
    u=[u feval];

    % Calculate the index of the input value with the highest evaluation value
    [maxv,ind]=max(feval);
    % Return the input value with the highest % rated value
    u=u(ind,1:2)';
end