function [ result ] = testOptimalSolution( WOpt, WIni, redThreshold, cm, tspan, mechSystem, mechOpt, basisFunc, basisOpt, cols )
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


trajectoryName = {'InitialSigmoid','Optimal', 'Intrinsic'};
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
    %uFunc = @(t)WHat*basisFunc(t,basisOpt);

    sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

    %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
    %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
    vopt = odeset('AbsTol',1e-5, 'RelTol',5e-7);
    %
    
    %tspanFull = linspace(tspan(1),tspan(2), 1000);
    
    [t, x] = ode15s(sys,tspan,xi,vopt);
    
    fprintf('Empirical Reduction of %s Trajectory\n', trajectoryName{i});
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
end

i = 3;
fprintf('\nTesting Intrinsic System dimensionality\n');

tempOpt.order = mechOpt.inputDim-1;
[ score, redOrder, redSys, hsv, normHsv ] =  evaluateDimCost_NBT_HSV2Cost(ones(mechOpt.inputDim),tspan,mechSystem, mechOpt, tempOpt, redThreshold, cm);


%[ redOrder, redSys, hsv , normHsv] = nonlinearBalancing( mechDirectSystem, mechOpt, redThreshold);
result(i).score = score;
result(i).hsv = hsv;
result(i).normHsv = hsv;
result(i).redOrder  = redOrder;
result(i).redSys = redSys;
redOrderList(i) = redOrder;
redScoreList(i) = score;

if(exist('cols','var')==0)
    cols = colormap(hsv(4)); %['r','b','g','k'];
else
    cols = [0.8,0.8,0.8 ; repmat(cols,3,1)];
end
figN = 2; %figure;

%% Plotting the results
for i = 1:2%3    
       
       if(i<3)
           figure(figN);
            subplot(2,1,1);
            p = plot(result(i).t,result(i).x(:,1:2),'color', cols(i,:),'linewidth',2); hold on;
            ylabel('Joint Angles \theta (rads)');
            xlabel('time t(secs)');
            legend('q_1','q_2');
    % 
            pGroup = hggroup;
            set(p,'Parent',pGroup);
            set(get(get(pGroup,'Annotation'),'LegendInformation'),...
            'IconDisplayStyle','on'); 
            
            subplot(2,1,2);
            %title('inputs computed by polybasis');
            
           
           p = plot(result(i).t,result(i).WHat*basisFunc(result(i).t,basisOpt),'color', cols(i,:),'linewidth',2);hold on;%, tTrain,result(train).xD,'k'); hold on;
           xlabel('time t(secs)');
           ylabel('Basis Computed Torques \tau_i (Nm)');
            
           pGroup = hggroup;
            set(p,'Parent',pGroup);
            set(get(get(pGroup,'Annotation'),'LegendInformation'),...
            'IconDisplayStyle','on'); 
            
        
      %  figure(figN+1);
           trajectoryTrace_snapshot_noRobot(fkin(result(i).x(:,1),result(i).x(:,2),robotData()),result(i).t,robotData(),figN+1,0,cols(i,:));hold on;
           
           
%              result{i}.cartTraj = cartend(1:end,3:4);
%    result{i}.straightEndVel = straightendVel:
    
           figure(figN+2);
           p = plot(result(i).t./result(i).t(end),result(i).cartTraj','color', cols(i,:),'linewidth',2); hold on;
           xlabel('time t/t_d(secs)');
           ylabel('Cartesian End Position P(m)');
           
           pGroup = hggroup;
            set(p,'Parent',pGroup);
            set(get(get(pGroup,'Annotation'),'LegendInformation'),...
            'IconDisplayStyle','on'); 
            
           
           figure(figN+3);
           
           p = plot(result(i).t(1:end-1)./result(i).t(end-1),result(i).cartEndVel,'color', cols(i,:),'linewidth',2); hold on;
           %plot(result(i).t(1:end-1),result(i).straightEndVel,cols{i}); hold on;
           xlabel('time t/t_d(secs)');
           ylabel('EndPoint Velocity V(m/sec)');
           
           pGroup = hggroup;
            set(p,'Parent',pGroup);
            set(get(get(pGroup,'Annotation'),'LegendInformation'),...
            'IconDisplayStyle','on'); 
            
       end
       
        figure(figN+4);
        p = plot(1:mechOpt.stateDim,cumsum(result(i).hsv)./sum(result(i).hsv),'-o','color', cols(i,:),'linewidth',2); hold on;
        axis tight;
        title('Hankel Singular Values');
        xlabel('State');
        ylabel('Normalised HSV');
        
        pGroup = hggroup;
        set(p,'Parent',pGroup);
        set(get(get(pGroup,'Annotation'),'LegendInformation'),...
        'IconDisplayStyle','on'); 
            
     


end

   
figure(figN); subplot(2,1,1); legend(trajectoryName{1:2});subplot(2,1,2); legend(trajectoryName{1:2}); hold on; grid on;
figure(figN+1);% legend(trajectoryName{1:end});
figure(figN+2); legend(trajectoryName{1:2});hold on;grid on;
figure(figN+3); legend(trajectoryName{1:2});hold on;grid on;
figure(figN+4); legend(trajectoryName{1:end});plot(cumsum(ones(mechOpt.stateDim,1)),redThreshold*ones(mechOpt.stateDim,1),'k','linewidth',2);hold on;grid on;

% figure(4);
% bar(1:length(trainingFunc)+1, redOrderList);
% %legend(trajectoryName{1:end});
% xlabel('Trajectories');
% ylabel('Reduced Dimensionality');

%disp(redOrderList)
%disp(redScoreList);

figure(figN+5); hold on;
bar(1:length(WOpt), [WIni, WOpt]);
xlabel('Weights Computed');
ylabel('Magnitude');
legend(trajectoryName{1:2});

% 
% figure;
% plot(result.t(1:end-1), straightend);
% xlabel('t (sec)');
% ylabel('Endpoint Velocity (m/sec)');
