function [ uFunc ] = generateBenchmarkTrajectories( xEnd, tspan,trajectoryType, opt,cols )
%GENERATEBENCHMARKTRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here
%GENERATERANDOMTRAJECTORY generates splines and function handles to
%its 1st and 2nd derivatives which fit xStart, xEnd and numPoints randomly
%chosen positions in between - aimed at the rigid kinematic chain system
%   Detailed explanation goes here

% TrajectoryType
% 1. Straight line sigmoidal
% 2. Polynomial curve
% 3. S shaped curve
% 4. overshooting curve
% 5. Spiral
   xStart = [0,-0.8]';     


    miny(1) = min([xStart(1),xEnd(1)]);
    maxy(1) = max([xStart(1),xEnd(1)]);
    miny(2) = min([xStart(2),xEnd(2)]);
    maxy(2) = max([xStart(2),xEnd(2)]);
    
       
    numPoints = 1;
    spy=zeros(2,numPoints+2);
    spyC=spy;
    
    switch(trajectoryType)
        case 1 
            tspanl = tspan(1):0.5:tspan(2);
            numPoints = length(tspanl);
            spy=zeros(2,numPoints);
            spyC=spy;
            fprintf('Straight Line');

            spyC(1,:) = miny(1) + 0.5*(maxy(1) - miny(1))*(1+tanh ( tspanl - 0.5*max(tspanl)));
            spyC(2,:) = miny(2) + 0.5*(maxy(2) - miny(2))*(1+tanh ( tspanl - 0.5*max(tspanl)));
            numPoints = numPoints-2;
        case 2
            fprintf('Overshoot Curve');
            spyC(:,2) = [0.55;0.45].*(xEnd + xStart);
             numPoints = 2;
             spy=zeros(2,numPoints+2);
             spyC=spy;
             fprintf('S Shaped Curve');
              spyC(:,2) = [0.33;0.35].*(xEnd + xStart);                        
              spyC(:,3) = [0.66;0.47].*(xEnd + xStart);                        
        case 4                         
            numPoints = 2;
            spy=zeros(2,numPoints+2);
            spyC=spy;
            fprintf('Curved Polynomial');
            spyC(:,2) = [0.33;0.47].*(xEnd + xStart);
            spyC(:,3) = [0.66;0.4305].*(xEnd + xStart);
        case 5          
            tspanl = tspan(1):0.75:tspan(2);
            numPoints = length(tspanl);
            spy=zeros(2,numPoints);
            spyC=spy;
            fprintf('Sigmoidal Line');
            spyC(1,:) = miny(1) + 0.5*(maxy(1) - miny(1))*(1+tanh ( tspanl - 0.5*max(tspanl)));
            spyC(2,:) = miny(2) + 0.5*(maxy(2) - miny(2))*(1+tanh ( tspanl - 0.5*max(tspanl)));
            numPoints = numPoints-2;
    end
    spx = linspace(tspan(1),tspan(2),length(spy));
     spy(:,1) = opt.initialCond(1:2);
     spyC(:,1) = [0,-0.8]';

    spyC(:,numPoints+2) = xEnd;
    spy(:,2:numPoints+2) = iKin(spyC(:,2:numPoints+2)','down',opt)';
    
    fprintf('Spy : ');
    disp(spy);
    
    plot(spyC(1,:),spyC(2,:),'linestyle','-','marker','o','color',cols); hold on;
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

   uFunc = @(t)invPlanar2dofArm([xd{1}(t) xd{2}(t)],[xDd{1}(t) xDd{2}(t)], [xDDd{1}(t) xDDd{2}(t)],opt)';

end

