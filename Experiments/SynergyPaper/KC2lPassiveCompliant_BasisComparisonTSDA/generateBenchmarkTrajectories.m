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
   xStart = [0.8,0]';%[0,-0.8]';     


    miny(1) = min([xStart(1),xEnd(1)]);
    maxy(1) = max([xStart(1),xEnd(1)]);
    miny(2) = min([xStart(2),xEnd(2)]);
    maxy(2) = max([xStart(2),xEnd(2)]);
    
       
    numPoints = 1;
    spy=zeros(2,numPoints+2);
    spyC=spy;
    
    switch(trajectoryType)
        case 1 
%             fprintf('Straight Line');
%             spyC(:,2) = 0.5* (xEnd + xStart) ;

            tspanl = linspace(tspan(1),tspan(2),3);
          %  tspanl2 = tspanl(2:end-1);
            numPoints = length(tspanl);
            spy=zeros(2,numPoints);
            spyC=spy;
            fprintf('Straight Line');
            %spyC(1,2:end-1) = miny(1) + 0.5*(maxy(1) - miny(1))*(1+tanh ( tspanl(2:end-1) - 0.5*max(tspanl)));
            %spyC(2,2:end-1) = miny(2) + 0.5*(maxy(2) - miny(2))*(1+tanh ( tspanl(2:end-1) - 0.5*max(tspanl)));
            spyC(1,:) = linspace(xStart(1),xEnd(1),length(tspanl));
            spyC(2,:) = linspace(xStart(2),xEnd(2),length(tspanl));
            %spyC = fitSigmoid(tspanl,xStart,xEnd);
            
            
            %fitSigmoid(tspanl,xStart,xEnd);%xStart(1) + 0.5*(xStart(1) - xEnd(1))*(1+tanh ( tspanl - 0.5*max(tspanl)));
            %spyC(2,:) = %xStart(2) + 0.5*(xStart(2) - xEnd(2))*(1+tanh ( tspanl - 0.5*max(tspanl)));
            %spyC(:,2) = [0.33;0.47].*(xEnd + xStart);
            %spyC(:,3) = [0.66;0.435].*(xEnd + xStart);
            %spyC(2,2) = 1.0*(xEnd(2));
            numPoints = numPoints-2;
        case 2
            fprintf('Overshoot Curve');
            spyC(:,2) = [0.45; 0.90].*(xEnd + xStart);%[0.5;0.47]   
           %numPoints = numPoints-1;
            %spyC(2,2) = 1.0*(xEnd(2));                       
            %spy(:,2) = 0.5* (xEnd - xStart) ;
        case 3 
             numPoints = 2;
             spy=zeros(2,numPoints+2);
             spyC=spy;
             fprintf('S Shaped Curve');
%             spyC=zeros(2,numPoints+2);
%             spyC(1,2) = 0.2*(xEnd(1) - xStart(2));                        
%             spyC(2,2) = 1.75*(xEnd(2));
%             spyC(1,3) = 0.60*(xEnd(1) - xStart(2));                        
%             spyC(2,3) = (xStart(2) + 0.5*(xStart(2) - xEnd(2)));
              spyC(:,2) = xStart + [0.3;0.45].*(xEnd - xStart);                        
              spyC(:,3) = xStart + [0.6;0.65].*(xEnd - xStart);   
              spy(:,3);
        case 4                         
            numPoints = 2;
            spy=zeros(2,numPoints+2);
            spyC=spy;
            fprintf('Curved Polynomial');
            spyC(:,2) = xStart + [0.33;0.37].*(xEnd - xStart);
            spyC(:,3) = xStart + [0.66;0.57].*(xEnd - xStart);
        case 5          
            tspanl = linspace(tspan(1),tspan(2),3);
          %  tspanl2 = tspanl(2:end-1);
            numPoints = length(tspanl);
            spy=zeros(2,numPoints);
            spyC=spy;
            fprintf('Sigmoidal Line');
            %spyC(1,2:end-1) = miny(1) + 0.5*(maxy(1) - miny(1))*(1+tanh ( tspanl(2:end-1) - 0.5*max(tspanl)));
            %spyC(2,2:end-1) = miny(2) + 0.5*(maxy(2) - miny(2))*(1+tanh ( tspanl(2:end-1) - 0.5*max(tspanl)));
            spyC = fitSigmoid(tspanl,xStart,xEnd);%miny(1) + 0.5*(maxy(1) - miny(1))*(1+tanh ( tspanl - 0.5*max(tspanl)));
            %spyC(:,2) = [0.33;0.47].*(xEnd + xStart);
            %spyC(:,3) = [0.66;0.435].*(xEnd + xStart);
            %spyC(2,2) = 1.0*(xEnd(2));
            numPoints = numPoints-2;
        %case '6'
         %   numPoints = 8;
    end
%     
    tt = linspace(tspan(1),tspan(2),length(spy));
	tTrain = [tt(1) 0.05*tt(end) tt(2:end-1) 0.95*tt(end) tt(end)]; 
    
     spy(:,1) = opt.initialCond(1:2);
     spyC(:,1) = [0.8,0.0]';%[0,-0.8]';

    %spy(1,2:end-1) = pointsy(1,:);%miny(1) + cumsum(rand(1,numPoints) * variance* (maxy(1) - miny(1)));
    %spy(2,2:end-1) = pointsy(1,:);%miny(2) + cumsum(rand(1,numPoints) * variance* (maxy(2) - miny(2)));

    spyC(:,numPoints+2) = xEnd;
    spy(:,2:numPoints+2) = iKin(spyC(:,2:numPoints+2)','down',opt)';
    
    fprintf('Spy : ');
    disp(spy);
    
    plot(spyC(1,:),spyC(2,:),'linestyle','-','marker','o','color',cols); hold on;
    %xlabel('Time (sec)');
    ylabel('Cartesian Position (m)');
    xlabel('Cartesian Position (m)');
   % figure;
    spline1 = csape(tTrain,[0 [spy(1,1) spy(1,:) spy(1,end)] 0], [2 2]);%spline(tTrain,[0 [spy(1,1) spy(1,:) spy(1,end)] 0]);
    spline2 = csape(tTrain,[0 [spy(2,1) spy(2,:) spy(2,end)] 0], [2 2]);%spline(tTrain,[0 [spy(2,1) spy(2,:) spy(2,end)] 0]);

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
%     
%         tt = linspace(tspan(1),tspan(2),length(spy));
% 	tTrain = [tt(1) 0.01*tt(end) tt(2:end-1) 0.99*tt(end) tt(end)]; 
%     
%      spy(:,1) = opt.initialCond(1:2);
%      spyC(:,1) = [0.8,0.0]';%[0,-0.8]';
% 
%     %spy(1,2:end-1) = pointsy(1,:);%miny(1) + cumsum(rand(1,numPoints) * variance* (maxy(1) - miny(1)));
%     %spy(2,2:end-1) = pointsy(1,:);%miny(2) + cumsum(rand(1,numPoints) * variance* (maxy(2) - miny(2)));
% 
%     spyC(:,numPoints+2) = xEnd;
%     spy(:,2:numPoints+2) = iKin(spyC(:,2:numPoints+2)','down',opt)';
%     
%     fprintf('Spy : ');
%     disp(spy);
%     
%     plot(spyC(1,:),spyC(2,:),'linestyle','-','marker','o','color',cols); hold on;
%     %xlabel('Time (sec)');
%     ylabel('Cartesian Position (m)');
%     xlabel('Cartesian Position (m)');
%    % figure;
%     spline1 = spline(tTrain,spy(1,:));
%     spline2 = spline(tTrain,spy(2,:));
% 
%     spline1D = fnder(spline1,1);
%     spline2D = fnder(spline2,1);
% 
%     spline1DD = fnder(spline1,2);
%     spline2DD = fnder(spline2,2);
% 
%     xd{1}= @(t)(ppval(spline1,t));
%     xd{2}= @(t)(ppval(spline2,t));
% 
%     xDd{1} = @(t)(ppval(spline1D,t));
%     xDd{2} = @(t)(ppval(spline2D,t));
% 
%     xDDd{1} = @(t)(ppval(spline1DD,t));
%     xDDd{2} = @(t)(ppval(spline2DD,t));
% 

   % uFunc{1} = @(t)(xDDd{1}(t) + opt.kx*xd{1}(t) + opt.bx*xDd{1}(t));
   % uFunc{2} = @(t)(xDDd{2}(t) + opt.ky*xd{2}(t) + opt.by*xDd{2}(t));
   uFunc = @(t)invPlanar2dofArm([xd{1}(t) xd{2}(t)],[xDd{1}(t) xDd{2}(t)], [xDDd{1}(t) xDDd{1}(t)],opt);
   
  figure(8);
   subplot(3,1,1); hold on;
   plot(newTime,uFunc(newTime'),'color',cols);axis tight;

end

