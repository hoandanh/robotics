%{
 This function will plot:
- model with a terrain map and obstacles
- solutions with different views
%}

function PlotSolution(sol,model,smooth)

    %% Plot 3D view
    figure(1)
    PlotModel(model)
    
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
    
    N = size(x_all,2); % real path length
    
    % given data in a point matrix, xyz, which is 3 x number of points
    xy = [x_all;y_all];
    [ndim,npts]=size(xy);
    xyp=zeros(size(xy));
    for k=1:ndim
       xyp(k,:)=ppval(csaps(1:npts,xy(k,:),smooth),1:npts);
    end
    plot(xyp(1,:),xyp(2,:),'k','LineWidth',2);

    % plot start point
    plot(x_all(1),y_all(1),'ks','MarkerSize',7,'MarkerFaceColor','k');
    % plot target point
    plot(x_all(N),y_all(N),'ko','MarkerSize',7,'MarkerFaceColor','k');

    % plot path
    plot(xyp(1,:),xyp(2,:),'k','LineWidth',2);
end