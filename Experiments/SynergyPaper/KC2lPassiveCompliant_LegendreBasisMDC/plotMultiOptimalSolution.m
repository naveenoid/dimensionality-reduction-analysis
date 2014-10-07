function [ result ] = plotMultiOptimalSolution( WOpt, WIni, redThreshold, cm, tspan, mechSystem, mechOpt, basisFunc, basisOpt ,cols )
%PLOTMULTIOPTIMALSOLUTION Summary of this function goes here
% TESTOPTIMALSOLUTION Function to plot the trajectory of the optimal weights
% solution computed by the optimization procedure
% Naveen Kuppuswamy (naveenoid@ifi.uzh.ch)


 
% % Robot setup
% mechOpt = robotData();
 xi = mechOpt.initialCond;
% tspan = [0;10];%[0:0.001:10];%train.tspan;
% mechSystem = @planar2dofArm;
% mechOpt.inputDim = 2;
% mechOpt.stateDim = 8;
% mechOpt.outputDim = 2;
% mechOpt.outputs = [1,2];

%cm = 1.75;
%redThreshold = 0.987;
% mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);
% 
% %polyBasis setup;
% basisFunc = @polyBasis;
% basisLearning = @polyBasisWeightLearning;
% basisOpt.order = 12;


%trajectoryName = {'P'};
clear result;
result= struct();
tTrain =(tspan(1):0.1:tspan(2))';

redOrderList = zeros(3,1);
redScoreList = zeros(3,1);


%xD = zeros(2,length(tTrain));
WHatStore(1).WHat = WIni;
WHatStore(2).WHat = WOpt;

%for i = 2%1:2
 i = 2;   
    WHat = zeros(size(WIni));
    if(size(WHatStore(i).WHat,2) == 1) 
        %Its organised as a column, presumably for optimisation, so reshape
        WHat = (reshape(WHatStore(i).WHat,[basisOpt.order+1,mechOpt.inputDim]))';
    else
        WHat = WHatStore(i).WHat;
    end

    mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
    mechBasisOpt = mechOpt;
    mechBasisOpt.inputDim = basisOpt.order+1;
    %uFunc = @(t)WHat*basisFunc(t,basisOpt);

    sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

    %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
    %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
    vopt = odeset('AbsTol',1e-5, 'RelTol',5e-7);
    %
    
    %tspanFull = linspace(tspan(1),tspan(2), 1000);
    
    [t, x] = ode15s(sys,tspan,xi,vopt);
    
   % fprintf('Empirical Reduction of %s Trajectory\n', trajectoryName{i});
    drawnow;
    
    result(i).WHat = WHat;

    result(i).t =t;
    result(i).x  =x;

    %[ score, redOrder, redSys, hsv , normHsv] =  evaluateDimCost_NBT(WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm);
     [ score, redOrder, redSys, hsv , normHsv] =  evaluateDimCost_NBT_HSV2Cost(WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm);

%disp(score)
%disp(redOrder)


    cartend = fkin(x(1:end,1),x(1:end,2),mechOpt);

    cartendDot = diff(cartend);
    cartendVel = cartendDot(:,3:4) ./ repmat(diff(t),1,2);
%     straightendVel = zeros(size(cartendDot,1),1);difft=diff(t);
%     for j = 1:size(cartendDot,1) 
%             straightendVel(j,:) = norm(cartendDot(j,3)-cartendDot(j,4))./difft(j);
%     end
    
 
    result(i).hsv = hsv;
    result(i).score = score;
    result(i).normHsv = normHsv;
    result(i).redOrder = redOrder;
    result(i).threshold = redThreshold;
    result(i).redSys = redSys;
    result(i).cartTraj = cartend(1:end,3:4);
    result(i).cartEndVel = cartendVel;
    
    redOrderList(i) = redOrder;
    redScoreList(i) = score;
%end
% 
% i = 3;
% fprintf('\nTesting Intrinsic System dimensionality\n');
% 
% tempOpt.order = mechOpt.inputDim-1;
% [ score, redOrder, redSys, hsv, normHsv ] =  evaluateDimCost_NBT_HSV2Cost(ones(mechOpt.inputDim),tspan,mechSystem, mechOpt, tempOpt, redThreshold, cm);
% 
% 
% %[ redOrder, redSys, hsv , normHsv] = nonlinearBalancing( mechDirectSystem, mechOpt, redThreshold);
% result(i).score = score;
% result(i).hsv = hsv;
% result(i).normHsv = hsv;
% result(i).redOrder  = redOrder;
% result(i).redSys = redSys;
% redOrderList(i) = redOrder;
% redScoreList(i) = score;

%cols = {'r','b','g','k'};

figN =  0;

%% Plotting the results
%for i = 2%1:3    
       
%        if(i<3)
%            figure(figN);
%             subplot(2,1,1);
%             p = plot(result(i).t,result(i).x(:,1:2),cols); hold on;
%             ylabel('Joint Angles \theta (rads)');
%             xlabel('time t(secs)');
%             legend('q_1','q_2');
%     % 
%             pGroup = hggroup;
%             set(p,'Parent',pGroup);
%             set(get(get(pGroup,'Annotation'),'LegendInformation'),...
%             'IconDisplayStyle','on'); 
%             
%             subplot(2,1,2);
%             %title('inputs computed by polybasis');
%             
%            
%            p = plot(result(i).t,result(i).WHat*basisFunc(result(i).t,basisOpt),cols);hold on;%, tTrain,result(train).xD,'k'); hold on;
%            xlabel('time t(secs)');
%            ylabel('Basis Computed Torques \tau_i (Nm)');
%             
%            pGroup = hggroup;
%             set(p,'Parent',pGroup);
%             set(get(get(pGroup,'Annotation'),'LegendInformation'),...
%             'IconDisplayStyle','on'); 
            
        
      %  figure(figN+1);
           %trajectoryTrace_snapshot(fkin(result(i).x(:,1),result(i).x(:,2),robotData()),result(i).t,robotData(),figN+1,0,cols);hold on;
           trajectoryTrace_snapshot_noRobot(fkin(result(i).x(:,1),result(i).x(:,2),robotData()),result(i).t,robotData(),figN+1,0,cols);hold on;
          
           axis tight; axis equal; grid on;
%              result{i}.cartTraj = cartend(1:end,3:4);
%    result{i}.straightEndVel = straightendVel:
    
           figure(figN+2);
           subplot(2,1,1);p = plot(result(i).t./result(i).t(end),result(i).cartTraj(:,1)','color',cols,'LineWidth',2.0); hold on;axis tight; grid on;annotatePlotGroup(p); 
           ylabel('Position P_x(m)','FontSize',14);
           subplot(2,1,2);p = plot(result(i).t./result(i).t(end),result(i).cartTraj(:,2)','color',cols,'LineWidth',2.0); hold on;axis tight; grid on;annotatePlotGroup(p); 
           ylabel('Position P_y(m)','FontSize',14);
           xlabel('Nomalised Time t/t_d (secs)','FontSize',14);
           
           
           %axis tight; grid on;annotatePlotGroup(p); 
            
           
           figure(figN+3);
           subplot(2,1,1);p = plot(result(i).t(1:end-1)./result(i).t(end-1),result(i).cartEndVel(:,1),'color',cols,'LineWidth',2.0); hold on;axis tight; grid on;annotatePlotGroup(p); 
           ylabel('Velocity dP_x/dt (m/sec)','FontSize',14);
           subplot(2,1,2);p = plot(result(i).t(1:end-1)./result(i).t(end-1),result(i).cartEndVel(:,2),'color',cols,'LineWidth',2.0); hold on;axis tight; grid on;annotatePlotGroup(p); 
           ylabel('Velocity dP_y/dt (m/sec)','FontSize',14);
           %plot(result(i).t(1:end-1),result(i).straightEndVel,cols{i}); hold on;
           xlabel('Nomalised Time t/t_d (secs)','FontSize',14);
           
           
           %axis tight; grid on;
           %annotatePlotGroup(p);
            
      % end
       
%         figure(figN+4);
%         p = plot(1:mechOpt.stateDim,cumsum(result(i).hsv)./sum(result(i).hsv),strcat(cols,'-o')); hold on;
%         axis tight;
%         title('Hankel Singular Values');
%         xlabel('State');
%         ylabel('Normalised HSV');
%         
%         pGroup = hggroup;
%         set(p,'Parent',pGroup);
%         set(get(get(pGroup,'Annotation'),'LegendInformation'),...
%         'IconDisplayStyle','on'); 
            
     


%end

   
%figure(figN); subplot(2,1,1); legend(trajectoryName{1:2});subplot(2,1,2); legend(trajectoryName{1:2});
%figure(figN+1);% legend(trajectoryName{1:end});
%figure(figN+2); legend(trajectoryName{1:2});
%figure(figN+3); legend(trajectoryName{1:2});
%figure(figN+4); legend(trajectoryName{1:end});plot(cumsum(ones(mechOpt.stateDim,1)),redThreshold*ones(mechOpt.stateDim,1),'k');

% figure(4);
% bar(1:length(trainingFunc)+1, redOrderList);
% %legend(trajectoryName{1:end});
% xlabel('Trajectories');
% ylabel('Reduced Dimensionality');

%disp(redOrderList)
%disp(redScoreList);

% figure;
% bar(1:length(WOpt), [WIni, WOpt]);
% xlabel('Weights Computed');
% ylabel('Magnitude');
% legend(trajectoryName{1:2});

% 
% figure;
% plot(result.t(1:end-1), straightend);
% xlabel('t (sec)');
% ylabel('Endpoint Velocity (m/sec)');

end

