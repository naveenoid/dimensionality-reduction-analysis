function [ result ] = testOptimalSolutionPoly( WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt)
% TESTOPTIMALSOLUTION Function to plot the trajectory of the optimal weights
% solution computed by the optimization procedure
% 
%
% Naveen Kuppuswamy (naveenoid@ifi.uzh.ch)
%

% 
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

if(exist('inCol','var')==0)
    inCol = 'b';
end


trajectoryName = {'Step','Optimal', 'Intrinsic'};
clear result;
result= struct();
tTrain =(tspan(1):0.1:tspan(2))';

redOrderList = zeros(3,1);
redScoreList = zeros(3,1);


%xD = zeros(2,length(tTrain));
WHatStore(1).WHat = WIni;
WHatStore(2).WHat = WOpt;

for i = 1:2
    
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
    
    sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

    %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
    %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
    vopt = odeset('AbsTol',1e-5, 'RelTol',5e-7);
    %
    
    [t, x] = ode15s(sys,tspan,xi,vopt);
    
    fprintf('Empirical Reduction of %s Trajectory\n', trajectoryName{i});
    drawnow;
    
    result(i).WHat = WHat;

    result(i).t =t;
    result(i).x  =x;

    
%% testing pure mech system 
tempOpt.order = mechOpt.inputDim-1;
   
    [score, redOrder , redSys, hsv, normHsv ] = evaluateDimCost_BT_HSV2Cost(ones(mechOpt.inputDim),mechOpt,tempOpt,redThreshold,[],[],[],1);

    cartend =  mechOpt.C*x(1:end,:)';
   
    cartendDot = diff(cartend);
    straightendVel = zeros(size(cartendDot,1),1);difft=diff(t);
    cartTraj =  zeros(size(cartendDot,1),2);
    for j = 1:size(cartendDot,1) 
            straightendVel(j,:) = norm(cartendDot(j,3)-cartendDot(j,4))./difft(j);
            cartTraj (j,:) = cartendDot(j,3)
    end
    
 
    result(i).hsv = hsv;
    result(i).score = score;
    result(i).normHsv = normHsv;
    result(i).redOrder = redOrder;
    result(i).threshold = redThreshold;
    result(i).redSys = redSys;
    result(i).xVel = x(1:end,3:4);%cartendDot./difft;%cartend(1:end,3:4);
    result(i).straightEndVel = straightendVel;
    
    redOrderList(i) = redOrder;
    redScoreList(i) = score;
end

i = 3;
fprintf('\nTesting Intrinsic System dimensionality\n');

tempOpt.order = mechOpt.inputDim-1;
[score, redOrder , redSys, hsv, normHsv ] = evaluateDimCost_BT_HSV2Cost(ones(mechOpt.inputDim),mechOpt,tempOpt,redThreshold,[],[],[],1);

result(i).score = score;
result(i).hsv = hsv;
result(i).normHsv = hsv;
result(i).redOrder  = redOrder;
result(i).redSys = redSys;
% redOrderList(i) = redOrder;
% redScoreList(i) = score;
% 
% cols = {'r','b','g','k'};
% 
% figN =  1;%figure;
% 
% 
% %% Plotting the results
% %for i = 1:3    
%        
%  %      if(i<3)
%            figure(figN);
%            % subplot(2,1,1);
%             p1 = plot(result(1).t./result(1).t(end),result(1).x(:,1:2),cols{1}); hold on;
%             p2 = plot(result(2).t./result(2).t(end),result(2).x(:,1:2),'color',inCol,'lineWidth',2);
%             ylabel('End Position (m)');
%             xlabel('Time t(secs)');
%             
%             p1Group = hggroup;
%             p2Group = hggroup;
%             set(p1,'Parent',p1Group);
%             set(p2,'Parent',p2Group) ;
%             
%             set(get(get(p1Group,'Annotation'),'LegendInformation'),...
%             'IconDisplayStyle','on'); 
%             set(get(get(p2Group,'Annotation'),'LegendInformation'),...
%             'IconDisplayStyle','on'); 
%                         
%         
%         figure(figN+1);
%            resPos(1).res = mechOpt.C*result(1).x';resPos(2).res = mechOpt.C*result(2).x';
%          
%            p1 = plot(resPos(1).res(1,:),resPos(1).res(2,:),strcat('--',cols{1}),...
%                 resPos(1).res(1,end),resPos(1).res(2,end),strcat('o',cols{1}),'lineWidth',2); hold on;
%               p2 = plot(resPos(2).res(1,:),resPos(2).res(2,:),'--',...%,'color',inCol,...
%                 resPos(2).res(1,end),resPos(2).res(2,end),'o','MarkerEdgeColor',inCol,'lineWidth',2);%,'color',inCol,'lineWidth',2);
%            
%            
%            p1Group = hggroup;
%            p2Group = hggroup;
%            set(p1,'Parent',p1Group);
%            set(p2,'Parent',p2Group) ;
%             
%            set(get(get(p1Group,'Annotation'),'LegendInformation'),...
%            'IconDisplayStyle','on'); 
%            set(get(get(p2Group,'Annotation'),'LegendInformation'),...
%            'IconDisplayStyle','on'); 
%        
%            figure(figN+2);
%            
% 
%            p1 = plot(result(1).t./result(1).t(end),result(1).xVel,cols{1}); hold on;
%            p2 = plot(result(2).t./result(2).t(end),result(2).xVel,'color',inCol,'lineWidth',2); hold on;
%            
%            p1Group = hggroup;
%            p2Group = hggroup;
%            set(p1,'Parent',p1Group);
%            set(p2,'Parent',p2Group) ;
%             
%            set(get(get(p1Group,'Annotation'),'LegendInformation'),...
%            'IconDisplayStyle','on'); 
%            set(get(get(p2Group,'Annotation'),'LegendInformation'),...
%            'IconDisplayStyle','on');
%            
%            xlabel('time t(secs)');
%            ylabel('EndPoint Velocity V(m/sec)');
%   %     end
%        
%         figure(figN+3);
%         p1 = plot(1:mechOpt.stateDim,cumsum(result(1).hsv)./sum(result(1).hsv),strcat(cols{1},'-o')); hold on;
%         p2 = plot(1:mechOpt.stateDim,cumsum(result(2).hsv)./sum(result(2).hsv),strcat(cols{2},'-o'));        
%         hold on;
%            
%         p1Group = hggroup;
%         p2Group = hggroup;
%         set(p1,'Parent',p1Group);
%         set(p2,'Parent',p2Group) ;
%             
%         set(get(get(p1Group,'Annotation'),'LegendInformation'),...
%         'IconDisplayStyle','on'); 
%         set(get(get(p2Group,'Annotation'),'LegendInformation'),...
%         'IconDisplayStyle','on');
%            
%         
%         axis tight;
%         title('Hankel Singular Values');
%         xlabel('State');
%         ylabel('Normalised HSV');
%  
% %end
% 
% figure(figN); %subplot(2,1,1); 
% legend(trajectoryName{1:2},'Location','SouthEast');grid on;
%  legend(trajectoryName{1:end},'Location','SouthEast'); 
% plot(0,0,'ko','MarkerSize',7,'LineWidth',2.5);plot(resPos(2).res(1,end),resPos(2).res(2,end),'ko','MarkerSize',7,'LineWidth',2.5);
% grid on; xlabel('x Position (m)');ylabel('y Position (m)');
% figure(figN+2); legend(trajectoryName{1:2}); grid on;
% figure(figN+3); legend(trajectoryName{1:end-1});plot(cumsum(ones(mechOpt.stateDim,1)),redThreshold*ones(mechOpt.stateDim,1),'k');
% grid on;
% 
% 
% 
% 
% figure(figN+4);
% bar(1:length(WOpt), [WIni, WOpt]);
% xlabel('Weights Computed');
% ylabel('Magnitude');
% legend(trajectoryName{1:2});

