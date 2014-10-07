function [ uFunc ] = generateBenchmarkTrajectories( xStart, xEnd, tspan,trajectoryType, opt,cols )
%GENERATEBENCHMARKTRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here
%GENERATERANDOMTRAJECTORY generates splines and function handles to
%its 1st and 2nd derivatives which fit xStart, xEnd and numPoints randomly
%chosen positions in between
%   Detailed explanation goes here

% TrajectoryType
% 1. Straight line sigmoidal
% 2. Polynomial curve
% 3. S shaped curve
% 4. overshooting curve
% 5. Spiral
    miny(1) = min([xStart(1),xEnd(1)]);
    maxy(1) = max([xStart(1),xEnd(1)]);
    miny(2) = min([xStart(2),xEnd(2)]);
    maxy(2) = max([xStart(2),xEnd(2)]);
    
       
    numPoints = 1;
    spy=zeros(2,numPoints+2);
    
    switch(trajectoryType)
        case 1 
            fprintf('Straight Line');
            spy(:,2) = 0.5* (xEnd - xStart) ;
        case 2
            fprintf('Overshoot Curve');
            spy(1,2) = 1.5*(xEnd(1) - xStart(1));                        
            spy(2,2) = 1.0*(xEnd(2));                       
        case 3 
            numPoints = 2;
            fprintf('S Shaped Curve');
            spy=zeros(2,numPoints+2);
            spy(1,2) = 0.2*(xEnd(1) - xStart(2));                        
            spy(2,2) = 1.75*(xEnd(2));
            spy(1,3) = 0.60*(xEnd(1) - xStart(2));                        
            spy(2,3) = (xStart(2) + 0.5*(xStart(2) - xEnd(2)));
        case 4                         
            fprintf('Curved Polynomial');
            spy(1,2) = 0.35*(xEnd(1) - xStart(1));                        
            spy(2,2) = 1.0*(xEnd(2));
        %case '6'
         %   numPoints = 8;
    end
%     
    spx = linspace(tspan(1),tspan(2),length(spy));
%  
     spy(:,1) = xStart;

    %spy(1,2:end-1) = pointsy(1,:);%miny(1) + cumsum(rand(1,numPoints) * variance* (maxy(1) - miny(1)));
    %spy(2,2:end-1) = pointsy(1,:);%miny(2) + cumsum(rand(1,numPoints) * variance* (maxy(2) - miny(2)));

    spy(:,end) = xEnd;
    fprintf('Spy : ');
    disp(spy);
    
    plot(spx(1,:),spx(1,:),'linestyle','-','marker','o','color',cols); hold on;
    %xlabel('Time (sec)');
    ylabel('Cartesian Position (m)');
    xlabel('Cartesian Position (m)');
   % figure;
    spline1 = spline(spx,spy(1,:));
    spline2 = spline(spx,spy(2,:));

    spline1D = fnder(spline1,1);
    spline2D = fnder(spline2,1);

    spline1DD = fnder(spline1,2);
    spline2DD = fnder(spline2,2);

    xd{1}= @(t)(ppval(spline1,t));
    xd{2}= @(t)(ppval(spline2,t));

    xDd{1} = @(t)(ppval(spline1D,t));
    xDd{2} = @(t)(ppval(spline2D,t));

    xDDd{1} = @(t)(ppval(spline1DD,t));
    xDDd{2} = @(t)(ppval(spline2DD,t));

    uFunc{1} = @(t)(xDDd{1}(t) + opt.kx*xd{1}(t) + opt.bx*xDd{1}(t));
    uFunc{2} = @(t)(xDDd{2}(t) + opt.ky*xd{2}(t) + opt.by*xDd{2}(t));

  figure(8); tP = linspace(tspan(1), tspan(2),500);
   subplot(3,1,1); hold on;
   plot(tP,uFunc{1}(tP),'color',cols, tP,uFunc{2}(tP),'color',cols);axis tight;

end

