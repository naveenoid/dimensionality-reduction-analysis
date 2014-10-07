function [ uFunc ] = generateBenchmarkTrajectoriesCartesian( xStart, xEnd, tspan,trajectoryType, opt,cols )
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
   %pts = fkin(opt.initialCond(1),opt.initialCond(2),opt);%[0.8,0]';%[0,-0.8]';     
   %xStart = pts(3:4)';

%     miny(1) = min([xStart(1),xEnd(1)]);
%     maxy(1) = max([xStart(1),xEnd(1)]);
%     miny(2) = min([xStart(2),xEnd(2)]);
%     maxy(2) = max([xStart(2),xEnd(2)]);
    
       
    numPoints = 1;
    spy=zeros(1,numPoints+2);
    spyC=spy;
    
    switch(trajectoryType)
        case 1 

            tspanl = linspace(tspan(1),tspan(2),3);
            numPoints = length(tspanl);
            spy=zeros(1,numPoints);
            spyC=spy;
        %    fprintf('Straight Line');
            spyC(1,:) = linspace(xStart(1),xEnd(1),length(tspanl));
            %spyC(2,:) = linspace(xStart(2),xEnd(2),length(tspanl));
            numPoints = numPoints-2;
        case 2
            fprintf('Overshoot Curve');
            spyC(1,2) = [0.5; 1.4].*(xEnd + xStart);%[0.5;0.47]   
        case 3 
             numPoints = 2;
             spy=zeros(1,numPoints+2);
             spyC=spy;
             fprintf('S Shaped Curve');
              spyC(1,2) = xStart +  [0.2;0.55].*(xEnd - xStart); %[0.3;0.55].*(xEnd - xStart);                        
              spyC(1,3) = xStart + [0.8;0.45].*(xEnd - xStart); %[0.6;0.55].*(xEnd - xStart);   
              spy(1,3);
        case 4                         
            numPoints = 2;
            spy=zeros(1,numPoints+2);
            spyC=spy;
            fprintf('Curved Polynomial');
            spyC(1,2) = xStart + [0.4;-0.25].*(xEnd - xStart);%[0.2;0.4].*(xEnd - xStart);%[0.33;0.37].*(xEnd - xStart);
            spyC(1,3) = xStart + [1.1;0.35].*(xEnd - xStart);%[0.8;0.9].*(xEnd - xStart);%[0.66;0.57].*(xEnd - xStart);
%         case 5          
%             tspanl = linspace(tspan(1),tspan(2),3);
%             numPoints = length(tspanl);
%             spy=zeros(2,numPoints);
%             spyC=spy;
%             fprintf('Sigmoidal Line');
%             spyC = fitSigmoid(tspanl,xStart,xEnd);%miny(1) + 0.5*(maxy(1) - miny(1))*(1+tanh ( tspanl - 0.5*max(tspanl)));
%             numPoints = numPoints-2;
    end
    spx = linspace(tspan(1),tspan(2),length(spy));
    % spy(:,1) = opt.initialCond(1:2);
    %  pts2 = fkin(opt.initialCond(1),opt.initialCond(2),opt);%[0.8,0.0]';%[0,-0.8]';
      spyC(1,1) = xStart;%pts2(3:4)';

    spyC(1,numPoints+2) = xEnd;
  % spy(:,2:numPoints+2) = iKin(spyC(:,2:numPoints+2)','down',opt)';
    
   % fprintf('Spy : ');
   % disp(spy);
    
%     plot(spyC(1,:),spyC(2,:),'linestyle','-','marker','o','color',cols,'linewidth',2); hold on;
%     ylabel('Cartesian Position (m)');
%     xlabel('Cartesian Position (m)');
%     axis tight;
%     grid on;
    
    spxNew = [spx(1) 0.1*spx(end) spx(2:end-1) 0.9*spx(end) spx(end)]; 
    spyCNew = [spyC(1:2,1) spyC spyC(1:2,end)];
    
    spline = csape(spxNew, [0 spyCNew(1,:) 0],[2 2]);
   % spline1 = csape(spx,spyC(1,:));
    %spline2 = csape(spxNew, [0 spyCNew(2,:) 0],[2 2]);
    %spline2 = csape(spx,spyC(2,:));
    

    splineD = fnder(spline,1);

    splineDD = fnder(spline,2);
    
    xd= @(t)(repmat(ppval(spline,t),opt.chainLength,1)); % chainlength X timelength
    %xd{2}= @(t)(ppval(spline2,t));

    xDd = @(t)(repmat(ppval(splineD,t),opt.chainLength,1)); % chainlength X timelength
    xDDd = @(t)(repmat(ppval(splineDD,t),opt.chainLength,1)); % chainlength X timelength
    uFunc = @(t)(((opt.massD)*(xDDd(t)' - opt.STIFF*xd(t)' - opt.DAMP*xDd(t)'))');
   
%    uFunc = @(t)invPlanar2dofArm([thetad{1}(t) thetad{2}(t)],[thetadD{1}(t) thetadD{2}(t)], [thetadDD{1}(t) thetadDD{2}(t)],opt);
%   
%     figure(8); tP = linspace(tspan(1), tspan(2),500);
%      %subplot(3,1,1); hold on;
%       plot(tP,uFunc{1}(tP),'color',cols); hold on;
%       plot(tP,uFunc{2}(tP),'color',cols);axis tight;
%     tP = linspace(tspan(1), tspan(2),500);
%     figure(7);
%       subplot(2,1,1);
%       plot(tP,xd{1}(tP),'color',cols); hold on;
%       plot(spxNew,spyCNew(1,:),'o','color',cols);
%       subplot(2,1,2);
%       plot(tP,xd{2}(tP),'color',cols); hold on;
%       plot(spxNew,spyCNew(2,:),'o','color',cols);
%     figure(8); 
%    %  subplot(3,1,1); hold on;
%       plot(tP,uFunc{1}(tP),'color',cols); hold on;
%       plot(tP,uFunc{2}(tP),'color',cols);axis tight;


end

