function config = Config()
    % Threats as cylinders
    threats = load('../Env/Env3.mat');

    % Start and end position
    start_location = [0,0];
    % End_location = [14,10];
    end_location = [14 14];
    
    config.alpha = readfis('dwa_alpha_anfis');
    config.beta = readfis('dwa_beta_anfis');
    config.gamma = readfis('dwa_gamma_anfis');
    
    % Incorporate map and searching parameters to a config
    config.start=start_location;
    config.end=end_location;
    config.threats = [threats.x;threats.y;threats.R]';
    config.initial_state = [0 0 pi/4 0 0]';
    
    % Robot mechanics model
    config.max_speed = 1.0;  % [m/s]
    config.min_speed = 0.0;  % [m/s]
    config.max_yaw_rate = toRadian(40.0);  % [rad/s]
    config.min_yaw_rate = -toRadian(40.0);  % [rad/s]
    config.max_accel = 0.2;  % [m/ss]
    config.max_delta_yaw_rate = toRadian(40.0);  % [rad/ss]
    config.v_resolution = 0.01;  % [m/s]
    config.yaw_rate_resolution = toRadian(1.0);  % [rad/s]
    
    config.dt = 0.1;  % [s] Time tick for motion prediction
    config.predict_time = 3.0;  % [s]
    
    config.robot_stuck_flag_cons = 0.001;  % constant to prevent robot stucked

    % Also used to check if goal is reached in both types
    config.robot_radius = 0.35;  % [m] for collision check
    config.roi = 5.0;
end