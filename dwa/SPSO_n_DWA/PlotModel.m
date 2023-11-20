% Plot the terrain model and threats
function PlotModel(model)
    meshgrid(model.X,model.Y); % Plot the data
    xlabel('x [m]');
    ylabel('y [m]');
    hold on
    grid on
    
    % Threats as cylinders
    threats = model.threats;
    plot(threats(:,1), threats(:,2),'o')
end