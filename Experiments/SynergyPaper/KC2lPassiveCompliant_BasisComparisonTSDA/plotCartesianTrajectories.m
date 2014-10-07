function [ output_args ] = plotCartesianTrajectories( result, resultMech, trajectoryName, xD, redThreshold, numBasis, numTestTraj, cols,mechOpt,costFuncHSV2Store,costFuncRedOrderStore,basisFunc, basisLearning, basisOpt, inputDim )
%PLOTCARTESIANTRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here

resultMechHSV = zeros(numTestTraj, mechOpt.stateDim);
for j = 1:numBasis
    for i = 1:numTestTraj
       % figure(2);
        set(0,'CurrentFigure',2);
%         hold on;;%+ 5*(j-1));
%         subplot(2,1,1);
%         p = plot(result(i,j).t,result(i,j).x(:,1:2),'color',cols(i,:)); hold on;
%         ylabel('Position');
%         xlabel('time t(secs)');
%         legend('q_1','q_2');
% 
%         annotatePlotGroup(p);
% 
%         %resPos = fk = fkin(x(end,1),x(end,2),mechOpt);fk(3:4)%mechOpt.C*result(i,j).x';
%         % 
%         subplot(2,1,2);
        %title('inputs computed by polybasis');
        p = plot(result(i,j).t,result(i,j).WHat*basisFunc{j}(result(i,j).t,basisOpt{j}),'color',cols(i,:));hold on;%, tTrain,result(train).xD,'k'); hold on;
        annotatePlotGroup(p);

         resultMechHSV(j,i,1:mechOpt.stateDim) = cumsum(result(i,j).hsv)./sum(result(i,j).hsv);
        
        set(0,'CurrentFigure',3);
        %figure(3); 
        hold on;%+ 5*(j-1));
        %trajectoryTrace_snapshot(fkin(result(i,j).x(:,1),result(i,j).x(:,2),robotData()),result(i,j).t,robotData(),3,0,cols(i,:));hold on;
        trajectoryTrace_snapshot_noRobot(fkin(result(i,j).x(:,1),result(i,j).x(:,2),robotData()),result(i,j).t,robotData(),3,0,cols(i,:));hold on;
        
        %p = plot(resPos(1,end),resPos(2,end),'o','color',cols(i,:)); hold on;
        %annotatePlotGroup(p,'off');
        %p = plot(resPos(1,:),resPos(2,:),'--','color',cols(i,:),'LineWidth',2.0); hold on;
        %annotatePlotGroup(p);

       % figure(4);
        set(0,'CurrentFigure',4);
        hold on;% + 5*(j-1));
        p = plot(1:mechOpt.stateDim,cumsum(result(i,j).hsv)./sum(result(i,j).hsv),'linestyle','-','marker','o','color',cols(i,:),'LineWidth',1.5);
        annotatePlotGroup(p); hold on;
        axis tight;
        title('Hankel Singular Values');
        xlabel('State');
        ylabel('Normalised HSV');
        
        
        
        set(0,'CurrentFigure',7);
            % %bar3(resultMechHSV);
            %     resultMechHSVTemp = squeeze(resultMechHSV(j,:,:))';
            %     resultMechHSVMod = resultMechHSVTemp(4:-1:1,:);%,resultMechHSVTemp(:,1)];
            %     bh = bar3(resultMechHSVMod);
            % 
            %     set(gca,'FontSize',12);
    
        resultMechHSVMod = squeeze(resultMechHSV(j,:,:));
    
        subplot(2,2,i);
        bar(resultMechHSVMod(i,:),'grouped');
        set(get(gca,'child'),'FaceColor',cols(i,:)); axis tight;
        %0.4,'FaceColor',cols(i,:)); axis tight;
        set(gca,'XTickLabel',{'s1','s2','s3','s4'});%{'T1','T2','T3','T4','T5'});
        set(gca,'yTick',fliplr([1,0.75,0.5,0.25,0]));
        grid on;
        set(gca,'FontSize',14);
        a = axis;
        
        kT = sum(resultMechHSVMod(i,:)<result(i,j).threshold);
        line([a(1); a(2)],[result(i,j).threshold ; result(i,j).threshold],'Color','k','LineWidth',2);
        
        tempTitle = sprintf('Trajectory T%d, k=%d',i,kT);%result(i,j).redOrder);
        title(tempTitle);
        if(i == 1 || i ==3)
            %ylabel( {'Normalised';'Magnitude'},'FontSize',14);
            ylabel('Magnitude','FontSize',14);
        end
        
        if(i>2)
            xlabel('HSV','FontSize',14);
        end
        
        
        set(0,'CurrentFigure',10+j);
        subplot(numTestTraj/2,numTestTraj/2,i);
        hintonDiagram(result(i,j).WHat,cols(i,:));
        tempTitle = sprintf('Trajectory T%d',i);%result(i,j).redOrder);
        title(tempTitle,'FontSize',14);
        
    end
end
set(0,'CurrentFigure',2);
%figure(1);
%legend(trajectoryName{1:end});
%figure(2); 
subplot(2,1,1);legend(trajectoryName{1:numTestTraj},'Location','SouthEast');subplot(2,1,2);legend(trajectoryName{1:numTestTraj},'Location','SouthEast');
%figure(3); 
set(0,'CurrentFigure',3);
legend(trajectoryName{1:numTestTraj},'Location','SouthWest'); %grid on;axis equal; axis tight;
%a = axis;axis(a*1.01); 
xlabel('P_x position (m)','FontSize',12);ylabel('P_y position (m)','FontSize',12);
set(gca,'FontSize',12);
plot(0,-0.8,'ko','MarkerSize',7,'LineWidth',2.5);plot(xD(1),xD(2),'ko','MarkerSize',7,'LineWidth',2.5);
set(0,'CurrentFigure',4);
%figure(4); 
p = plot(1:mechOpt.stateDim,cumsum(resultMech.hsv)./sum(resultMech.hsv),'k-o','LineWidth',2.0); hold on;
annotatePlotGroup(p);
p = plot(cumsum(ones(4,1)),redThreshold*ones(4,1),'k'); grid on;
annotatePlotGroup(p,'off');

legend([trajectoryName{1:numTestTraj},trajectoryName{end}]','Location','SouthEast');

set(0,'CurrentFigure',5);
 %figure(5);



%bar(1:2,costFuncStore');
% if(numTestTraj>1)
%     colormap(jet(numTestTraj+1));
%     bar(1:numTestTraj,costFuncHSV2Store(1:numTestTraj,:));
% else
%colormap(Bone(numTestTraj+2));
%subplot(2,1,2);
bar(costFuncHSV2Store(1:numTestTraj,:),'grouped','FaceColor',0.7*[1 1 1],'LineWidth',1.0);
set(gca,'xTick',1:numTestTraj,'xTickLabel',trajectoryName,'FontSize',12);
%legend('PolyBasis','Fourier Basis','Location','NorthWest');
axis tight; grid on;xlabel('Trajectory','FontSize',12);
ylabel('Cost J(\sigma _2)','FontSize',12);

set(gca,'FontSize',12);
set(0,'CurrentFigure',6);
% %subplot(2,1,1);
% bar(costFuncRedOrderStore(1:numTestTraj,:),0.5,'FaceColor',0.8*[1 1 1],'LineWidth',1.0);
% set(gca,'xTick',1:numTestTraj,'xTickLabel',trajectoryName);
% %legend('PolyBasis','Fourier Basis','Location','NorthWest');
% axis tight; grid on;xlabel('Trajectory','FontSize',12);
% ylabel('Dimensionality D_W','FontSize',12);
% set(gca,'FontSize',12);


set(0,'CurrentFigure',6);
subplot(1,2,2);
%colormap(gray);
bar(cumsum(resultMech.hsv)./sum(resultMech.hsv),'grouped','FaceColor',0.5*[1 1 1]);
axis tight;
 a = axis;
 kT = sum(resultMechHSVMod(i,:)<result(i,j).threshold);
 %line([a(1); a(2)],[result(i,j).threshold ; result(i,j).threshold],'Color','k','LineWidth',2);
 line([a(1); a(2)],[ 0.9 ; 0.9],'Color','k','LineWidth',2);

grid on; axis tight;
xlabel('HSV','FontSize',14);
ylabel('Normalised Magnitude','FontSize',14);
set(gca,'XTickLabel',{'s1','s2','s3','s4'},'FontSize',14);
title('HSV : Kinematic Chain', 'FontSize',14);

subplot(1,2,1);
%colormap(gray);
bar([0.8162 1.0 1.0 1.0]','grouped','FaceColor',0.5*[1 1 1]);
axis tight;
 a = axis;
 kT = sum(resultMechHSVMod(i,:)<result(i,j).threshold);
 %line([a(1); a(2)],[result(i,j).threshold ; result(i,j).threshold],'Color','k','LineWidth',2);
 line([a(1); a(2)],[ 0.9 ; 0.9],'Color','k','LineWidth',2);

grid on; axis tight;
xlabel('HSV','FontSize',14);
ylabel('Normalised Magnitude','FontSize',14);
set(gca,'XTickLabel',{'s1','s2','s3','s4'},'FontSize',14);
title('HSV : Tethered Mass', 'FontSize',14);

%end
%end

% 
% set(0,'CurrentFigure',7);
% % %bar3(resultMechHSV);
% %     resultMechHSVTemp = squeeze(resultMechHSV(j,:,:))';
% %     resultMechHSVMod = resultMechHSVTemp(4:-1:1,:);%,resultMechHSVTemp(:,1)];
% %     bh = bar3(resultMechHSVMod);
% % 
% %     set(gca,'FontSize',12);
%     
%         resultMechHSVMod = squeeze(resultMechHSV(j,:,:));
%     
%         subplot(2,2,i);
%         bar(resultMechHSVMod(i,:),'hist');
%         set(get(gca,'child'),'FaceColor',cols(i,:)); axis tight;
%         %0.4,'FaceColor',cols(i,:)); axis tight;
%         set(gca,'XTickLabel',{'s1','s2','s3','s4'});%{'T1','T2','T3','T4','T5'});
%         set(gca,'yTick',fliplr([1,0.75,0.5,0.25,0]));
%         grid on;
%         set(gca,'FontSize',14);
%         a = axis;
%         
%         kT = sum(resultMechHSVMod(i,:)<result(i,j).threshold);
%         line([a(1); a(2)],[result(i,j).threshold ; result(i,j).threshold],'Color','k','LineWidth',2);
%         
%         tempTitle = sprintf('Trajectory T%d, k=%d',i,kT);%result(i,j).redOrder);
%         title(tempTitle);
%         if(i == 1 || i ==3)
%             %ylabel( {'Normalised';'Magnitude'},'FontSize',14);
%             ylabel('Magnitude','FontSize',14);
%         end
%         
%         if(i>2)
%             xlabel('HSV','FontSize',14);

%colormap(hsv(numTestTraj+1));
%testT = 0:0.001:1.5;
% figure(6);
% plot(testT,basisFunc{1}(testT,basisOpt{1}),'LineWidth',2.0); grid on;
% xlabel('Time (sec)');
% ylabel('\Psi_i(t)');
% figure(7);
% plot(testT,basisFunc{2}(testT,basisOpt{2}),'LineWidth',2.0); grid on;
% xlabel('Time (sec)');
% ylabel('Basis Functions \Psi_i(t)');

figFolder = './Plots/SynergyPaper/KC2lPassiveCompliant_BasisComparisonTSDA/nonLinSys_';
%figure(3); 
set(0,'CurrentFigure',3);
figTitle = 'cartesianTrajectory';
print('-depsc2','-r800',strcat(figFolder,figTitle));

set(0,'CurrentFigure',5);
%figure(5); 
figTitle = 'cartesianHSVCost';
print('-depsc2','-r800',strcat(figFolder,figTitle));


set(0,'CurrentFigure',6);
%figure(5); 
figTitle = 'HSVIntrinsic_KC_TM';
print('-depsc2','-r800',strcat(figFolder,figTitle));



set(0,'CurrentFigure',7);
%figure(5); 
figTitle = 'HSVMagnitudes';
print('-depsc2','-r800',strcat(figFolder,figTitle));



set(0,'CurrentFigure',11);
%figure(5); 
figTitle = 'LegendreHinton';
print('-depsc2','-r800',strcat(figFolder,figTitle));



% figure(6); figTitle = 'polyBasis';
% print('-depsc2','-r800',strcat(figFolder,figTitle));
% 
% figure(7); figTitle = 'fourierBasis';
% print('-depsc2','-r800',strcat(figFolder,figTitle));
end