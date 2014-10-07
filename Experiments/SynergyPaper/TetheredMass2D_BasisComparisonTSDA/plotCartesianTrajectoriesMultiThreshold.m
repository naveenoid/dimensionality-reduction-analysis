function [ output_args ] = plotCartesianTrajectoriesMultiThreshold( result, resultMech, trajectoryName, xD, redThreshold, numBasis, numTestTraj, cols,mechOpt,costFuncStore,hsvCostFuncStore,basisFunc, basisLearning, basisOpt, inputDim )
%PLOTCARTESIANTRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here



for j = 1:numBasis
    for i = 1:numTestTraj
        figure(2);hold on;%+ 5*(j-1));
        subplot(2,1,1);
        p = plot(result(i,j).t,result(i,j).x(:,1:2),'color',cols(i,:)); hold on;
        ylabel('Position');
        xlabel('time t(secs)');
        legend('q_1','q_2');

        annotatePlotGroup(p);

        resPos = mechOpt.C*result(i,j).x';
        % 
        subplot(2,1,2);
        %title('inputs computed by polybasis');
        p = plot(result(i,j).t,result(i,j).WHat*basisFunc{j}(result(i,j).t,basisOpt{j}),'color',cols(i,:));hold on;%, tTrain,result(train).xD,'k'); hold on;
        annotatePlotGroup(p);

        
        
        
        figure(3); hold on;%+ 5*(j-1));
        %trajectoryTrace_snapshot(fkin(result.x(:,1),result.x(:,2),robotData()),result.t,robotData(),2,0);hold on;
        p = plot(resPos(1,end),resPos(2,end),'o','color',cols(i,:)); hold on;
        annotatePlotGroup(p,'off');
        p = plot(resPos(1,:),resPos(2,:),'--','color',cols(i,:),'LineWidth',2.0); hold on;
        annotatePlotGroup(p);

        figure(4); hold on;% + 5*(j-1));
        p = plot(1:mechOpt.stateDim,cumsum(result(i,j).hsv)./sum(result(i,j).hsv),'linestyle','-','marker','o','color',cols(i,:),'LineWidth',1.5);
        annotatePlotGroup(p); hold on;
        axis tight;
        title('Hankel Singular Values');
        xlabel('State');
        ylabel('Normalised HSV');
    end
end

figure(1); %legend(trajectoryName{1:end});
figure(2); subplot(2,1,1);legend(trajectoryName{1:end-1},'Location','SouthEast');subplot(2,1,2);legend(trajectoryName{1:end-1},'Location','SouthEast');
figure(3); legend(trajectoryName{1:end-1},'Location','SouthEast'); grid on;axis equal; a = axis;axis(a*1.1); xlabel('P_x position (m)');ylabel('P_y position (m)');

plot(0,0,'ko','MarkerSize',7,'LineWidth',2.5);plot(xD(1),xD(2),'ko','MarkerSize',7,'LineWidth',2.5);
figure(4); 
p = plot(1:mechOpt.stateDim,cumsum(resultMech.hsv)./sum(resultMech.hsv),'k-o','LineWidth',2.0); hold on;
annotatePlotGroup(p);
p = plot(cumsum(ones(4,1)),redThreshold*ones(4,1),'k'); grid on;
annotatePlotGroup(p,'off');

legend(trajectoryName{1:end},'Location','SouthEast');

 figure(5);


colormap(jet(numTestTraj+1));
%bar(1:2,costFuncStore');
subplot(2,1,1);bar(1:numTestTraj,hsvCostFuncStore(1:numTestTraj,:)); hold on;
ylabel('Dimensionality D_W');

legend('Polynomial Basis','Fourier Basis','Location','NorthWest');
set(gca,'xTick',1:numTestTraj+1,'xTickLabel',trajectoryName);

axis tight; grid on;
%ylabel('D_W');
xlabel('Trajectory');

subplot(2,1,2);bar(1:numTestTraj,costFuncStore(1:numTestTraj,:)); hold on;
ylabel('Cost J(\sigma _2)');
set(gca,'xTick',1:numTestTraj+1,'xTickLabel',trajectoryName);
legend('Polynomial Basis','Fourier Basis','Location','NorthWest');
%haxes = plotyy(1:numTestTraj,hsvCostFuncStore(1:numTestTraj,:),1:numTestTraj,costFuncStore(1:numTestTraj,:));

%axes(haxes(2));

%legend('Polynomial Basis','Fourier Basis','Location','NorthWest');
axis tight; grid on;
%ylabel('D_W');
xlabel('Trajectory');

%colormap(hsv(numTestTraj+1));
testT = 0:0.001:1.5;
figure(6);
basisOpt{1}.tmax = testT(end);
plot(testT,basisFunc{1}(testT,basisOpt{1}),'LineWidth',2.0); grid on;
xlabel('Time (sec)');
ylabel('\Psi_i(t)');
figure(7);
basisOpt{2}.tmax = testT(end);
plot(testT,basisFunc{2}(testT,basisOpt{2}),'LineWidth',2.0); grid on;
xlabel('Time (sec)');
ylabel('Basis Functions \Psi_i(t)');

figFolder = './Plots/PolyFourierBasisComparison/multi_';
figure(3); figTitle = 'cartesianTrajectory';
print('-depsc2','-r800',strcat(figFolder,figTitle));

figure(5); figTitle = 'cartesianRedDim';
print('-depsc2','-r800',strcat(figFolder,figTitle));

figure(6); figTitle = 'polyBasis';
print('-depsc2','-r800',strcat(figFolder,figTitle));

figure(7); figTitle = 'fourierBasis';
print('-depsc2','-r800',strcat(figFolder,figTitle));
end