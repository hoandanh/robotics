function dw=CalcDynamicWindow(x,config)
    dt = config.dt;
    % Compute DyamicWindow from model and current state
    % Window by vehicle model
    Vs=[0 config.max_speed -config.max_yaw_rate config.max_yaw_rate];

    % Window by motion model
    Vd=[x(4)-config.max_accel*dt x(4)+config.max_accel*dt...
        x(5)-config.max_delta_yaw_rate*dt x(5)+config.max_delta_yaw_rate*dt];

    % Final Dynamic Window Computation
    Vtmp=[Vs;Vd];
    dw=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
    %[vmin,vmax,ωmin,ωmax]
end