%
% Calculate path cost
%

function cost=MyCost(sol,model)
    
    J_inf = inf;
    
    % Input solution
    x=sol.x;
    y=sol.y;
    
    % Start location
    xs=model.start(1);
    ys=model.start(2);
    
    % Final location
    xf=model.end(1);
    yf=model.end(2);
    
    x_all = [xs x xf];
    y_all = [ys y yf];
    
    N = size(x_all,2); % Full path length
    %============================================
    % J1 - Cost for path length    
    J1 = 0;
    for i = 1:N-1
        diff = [x_all(i+1) - x_all(i);y_all(i+1) - y_all(i)];
        J1 = J1 + norm(diff);
    end

    %==============================================
    % J2 - threats/obstacles Cost   

    % Threats/Obstacles
    threats = model.threats;
    threat_num = size(threats,1);
    
    drone_size = 1;
    danger_dist = drone_size + 0.5;
    
    J2 = 0;
    for i = 1:threat_num
        threat = threats(i,:);
        threat_x = threat(1);
        threat_y = threat(2);
        threat_radius = threat(3);
        for j = 1:N-1
            % Distance between projected line segment and threat origin
            dist = DistP2S([threat_x threat_y],[x_all(j) y_all(j)],[x_all(j+1) y_all(j+1)]);
            if dist > (threat_radius + drone_size + danger_dist) % No collision
                threat_cost = 0;
            elseif dist < (threat_radius + drone_size)  % Collision
                threat_cost = J_inf;
            else  % danger
                threat_cost = (threat_radius + drone_size + danger_dist) - dist;
            end
            J2 = J2 + threat_cost;
        end
    end

    %==============================================
    % J3 - Smooth cost
    J3 = 0;
    turning_max = 45;
    for i = 1:N-2
        % Projection of line segments to Oxy ~ (x,y,0)
        for j = i:-1:1
             segment1_proj = [x_all(j+1); y_all(j+1); 0] - [x_all(j); y_all(j); 0];
             if nnz(segment1_proj) ~= 0
                 break;
             end
        end

        for j = i:N-2
            segment2_proj = [x_all(j+2); y_all(j+2); 0] - [x_all(j+1); y_all(j+1); 0];
             if nnz(segment2_proj) ~= 0
                 break;
             end
        end

        turning_angle = atan2d(norm(cross(segment1_proj,segment2_proj)),dot(segment1_proj,segment2_proj));
       
        if abs(turning_angle) > turning_max
            J3 = J3 + abs(turning_angle);
        end
    end

    %============================================
    % Weight coeffcients
    b1 = 5;
    b2 = 1;
    b3 = 1;
    % Overall cost
    cost = b1*J1 + b2*J2 + b3*J3;
end