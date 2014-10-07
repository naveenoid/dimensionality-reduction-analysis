function [ uFunc ] = generateRandomTrajectory( thetaStart, xEnd, tspan,numPoints, mechOpt )
%GENERATERANDOMTRAJECTORY generates splines and function handles to
%its 1st and 2nd derivatives which fit xStart, xEnd and numPoints randomly
%chosen positions in between
%   Detailed explanation goes here

    %thetaStart = iKin(xStart','up');
    thetaEnd = iKin(xEnd','down',mechOpt);%iKin(xDes','up',mechOpt)'

   % thetaEnd = thetaStart;
    

    miny(1) = min([thetaStart(1),thetaEnd(1)]);
    maxy(1) = max([thetaStart(1),thetaEnd(1)]);
    miny(2) = min([thetaStart(2),thetaEnd(2)]);
    maxy(2) = max([thetaStart(2),thetaEnd(2)]);

    spx = linspace(tspan(1),tspan(2),numPoints+2);
    spy = zeros(2,length(spx));
    spy(:,1) = thetaStart;

    spy(1,2:end-1) = miny(1) + rand(1,numPoints) * 1.5* (maxy(1) - miny(1));
    spy(2,2:end-1) = miny(2) + rand(1,numPoints) * 1.5* (maxy(2) - miny(2));

    spy(:,end) = thetaEnd;
    
    plot(spx,spy,'-o');
    figure;
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

    
    uFunc = @(t)invPlanar2dofArm([xd{1}(t) xd{2}(t)],[xDd{1}(t) xDd{2}(t)], [xDDd{1}(t) xDDd{1}(t)],mechOpt);
    %uFunc{1} = @(t)(xDDd{1}(t) + opt.kx*xd{1}(t) + opt.bx*xDd{1}(t));
    %uFunc{2} = @(t)(xDDd{2}(t) + opt.ky*xd{2}(t) + opt.by*xDd{2}(t));
    
    %uFunc = @(t)(zeros(length(t),2));
end

