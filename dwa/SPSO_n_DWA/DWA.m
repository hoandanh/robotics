function result = DWA(show_animation, config)
%     disp('Dynamic Window Approach sample program start!!')
    % Initial state of robot[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
    x=config.initial_state;
    % Goal position [x(m),y(m)]
    goal = config.end;
    % Obstacle list [x(m) y(m)]
    obstacle = config.threats;

    % Simulation area size [xmin xmax ymin ymax]
    area=[-2 17 -2 17];

    % Simulation result
    result.x=[];
    tic;
    if show_animation == 1
        movcount=0;
        vi = VideoWriter('movie.avi');
    end
    % Main loop
    for i=1:1000
        i
        % Calculation of input value by DWA
        [u,traj]=DynamicWindowApproach(x,config);
        % Motion config locomotion
        x=f(x,u,config.dt);

        % Save Simulation Results
        result.x=[result.x; x'];

        % Goal Judgment
        if norm(x(1:2)-goal') < 0.5
%             disp('Arrive Goal!!');
            break;
        end

        %====Animation====
        if show_animation == 1
            hold off;
            ArrowLength=0.5;% Arrow length

            % Robot
            quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
            plot(result.x(:,1),result.x(:,2),'-b');hold on;
            plot(goal(1),goal(2),'*r');hold on;
            plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
        end

        % Explore track representation
        if ~isempty(traj)
            for it=1:length(traj(:,1))/5
                ind=1+(it-1)*5;
                if show_animation == 1
                    plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
                end
            end
        end
        if show_animation == 1
            open(vi)
            axis(area);
            grid on;
            drawnow;
            movcount=movcount+1;
            frames = getframe(gcf);% get animation frames
            writeVideo(vi, frames);
        end
    end
    if show_animation == 1
        figure(2)
        plot(result.x(:,4));
        title('Velocity [m/s]');
        toc
%         movie2avi(mov,'movie.avi');
    end
end