function config = Config(varargin)
    if nargin == 0
        start_location = [0,0];
        end_location = [16,14];
    else
        start_location = varargin{1};
        end_location = varargin{2};
    end

    % Threats as cylinders
    R = [0.15,0.15,0.15,0.15,0.15,0.15,0.15,0.15,0.15,0.15,0.15,0.15,0.15,0.15,0.15];
    x = [-1,0,4,5,5,5,5,8,7,8,9,12,12,15,13];
    y = [-1,2,2,4,5,6,9,9,9,10,11,13,12,15,13];
    
    % Incorporate map and searching parameters to a config
    config.start=start_location;
    config.end=end_location;
    config.threats = [x;y;R]';
    config.initial_state = [start_location [pi/4 0 0]]';
    
    % Robot mechanics config
    config.max_speed = 1.0;  % [m/s]
    config.min_speed = -0.5;  % [m/s]
    config.max_yaw_rate = toRadian(40.0);  % [rad/s]
    config.max_accel = 0.2;  % [m/ss]
    config.max_delta_yaw_rate = toRadian(40.0);  % [rad/ss]
    config.v_resolution = 0.01;  % [m/s]
    config.yaw_rate_resolution = toRadian(1.0);  % [rad/s]
    
    config.dt = 0.1;  % [s] Time tick for motion prediction
    config.predict_time = 3.0;  % [s]
    
%     config.alpha = 0.5; % head
%     config.gamma = 0.45; % obs
%     config.beta = 0.375; % vel

    config.alpha = 0.25; % head
    config.gamma = 0.5; % obs
    config.beta = 0.375; % vel

    config.robot_stuck_flag_cons = 0.001;  % constant to prevent robot stucked

    % Also used to check if goal is reached in both types
    config.robot_radius = 1.0;  % [m] for collision check
    
    MAPSIZE_X = 30;
    MAPSIZE_Y = 30;
    [X,Y] = meshgrid(-2:MAPSIZE_X,-2:MAPSIZE_Y); % Create all (x,y) points to plot

    % Map limits
    xmin= 1;
    xmax= MAPSIZE_X;
    
    ymin= 1;
    ymax= MAPSIZE_Y;
    
    % Number of path nodes (not including the start position (start node))
    n=10;
    
    config.n=n;
    config.xmin=xmin;
    config.xmax=xmax;
    config.ymin=ymin;
    config.ymax=ymax;
    config.MAPSIZE_X = MAPSIZE_X;
    config.MAPSIZE_Y = MAPSIZE_Y;
    config.X = X;
    config.Y = Y;
end