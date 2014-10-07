function [ output_args ] = plotCartesianTrajectories( result, resultMech, trajectoryName, xD, redThreshold, numBasis, numTestTraj, cols,mechOpt,costFuncStore,hsvCostFuncStore,basisFunc, basisLearning, basisOpt, inputDim )
%PLOTCARTESIANTRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here


resultMechHSV = zeros(numTestTraj, mechOpt.stateDim);
barHandle = cell(2);

for j = 1:numBasis
    for i = 1:numTestTraj
%         set(0,'CurrentFigure',2);
      %  figure(2);
        hold on;%+ 5*(j-1));
%         subplot(2,1,1);
%         p = plot(result(i,j).t,result(i,j).x(:,1:2),'color',cols(i,:)); hold on;
%         ylabel('Position');
%         xlabel('time t(secs)');
%         legend('q_1','q_2');
% 
%         annotatePlotGroup(p);§

         resPos = mechOpt.C*result(i,j).x';
%         % 
%        % subplot(2,1,2);
%         %title('inputs computed by polybasis');
%         p = plot(result(i,j).t,result(i,j).WHat*basisFunc{j}(result(i,j).t,basisOpt{j}),'color',cols(i,:));hold on;%, tTrain,result(train).xD,'k'); hold on;
%         annotatePlotGroup(p);
% 
%         
        resultMechHSV(j,i,1:mechOpt.stateDim) = cumsum(result(i,j).hsv)./sum(result(i,j).hsv);
        
        set(0,'CurrentFigure',4); hold on;
       % figure(3); hold on;%+ 5*(j-1));
        %trajectoryTrace_snapshot(fkin(result.x(:,1),result.x(:,2),robotData()),result.t,robotData(),2,0);hold on;
        p = plot(resPos(1,end),resPos(2,end),'o','color',cols(i,:)); hold on;
        annotatePlotGroup(p,'off');
        p = plot(resPos(1,:),resPos(2,:),'--','color',cols(i,:),'LineWidth',2.0); hold on;
        annotatePlotGroup(p);

        %figure(4); 
        set(0,'CurrentFigure',5);
        hold on;% + 5*(j-1));
        p = plot(1:mechOpt.stateDim,cumsum(result(i,j).hsv)./sum(result(i,j).hsv),'linestyle','-','marker','o','color',cols(i,:),'LineWidth',1.5);
        annotatePlotGroup(p); hold on;
        axis tight;
        title('Hankel Singular Values');
        xlabel('State');
        ylabel('Normalised HSV');
        
        set(0,'CurrentFigure',2+j-1);
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
        if(numTestTraj>1)
            subplot(numTestTraj/2,numTestTraj/2,i);
        else
            subplot(1,1,1);
        end
        set(gca,'FontSize',14);
        hintonDiagram(result(i,j).WHat,cols(i,:));
        tempTitle = sprintf('Trajectory T%d',i);%result(i,j).redOrder);
        title(tempTitle,'FontSize',14);
        set(gca,'FontSize',14);

    end
    
  %  set(0,'CurrentFigure',2+j-1);
  %  colormap(hsv);
 %   subplot(1,numBasis,j);

 
 %   resultMechHSVMod = squeeze(resultMechHSV(j,:,:));
    %resultMechHSVMod = resultMechHSVTemp(4:-1:1,:);%,resultMechHSVTemp(:,1)];
 
%    resultMechHSVTemp = squeeze(resultMechHSV(j,:,:))';
 %   resultMechHSVMod = resultMechHSVTemp(4:-1:1,:);%,resultMechHSVTemp(:,1)];
    % bh = bar3(resultMechHSVMod);
    %h = bar(resultMechHSVMod,'grouped');
    
    
    %set(bh(1),'facecolor',cols(1,:));
    %set(bh(2),'facecolor',cols(2,:));
    %set(bh(3),'facecolor',cols(3,:));
    %set(bh(4),'facecolor',cols(4,:));
    
    
    %surf(1:numTestTraj, 1:mechOpt.stateDim,squeeze(resultMechHSV(j,:,:)));
   % contourf(1:numTestTraj, 1:mechOpt.stateDim,squeeze(resultMechHSV(j,:,:))); axis tight;  grid on;
%    set(0,'CurrentFigure',2);
 %   subplot(2,numBasis,j);
  %  surf(squeeze(resultMechHSV(j,:,:)));

   % %bar3(squeeze(resultMechHSV(j,:,:)));

end

%figure(1); 
%set(0,'CurrentFigure',1);-1
%legend(trajectoryName{1:end});
%figure(2); 

% set(0,'CurrentFigure',2);
% %subplot(1,2,1);
%   set(gca,'XTickLabel',{'T1','T2','T3','T4','T5'});
%   set(gca,'YTickLabel',{'s1','s2','s3','s4'});
%   
%     xlabel('Trajectory');
%     ylabel('HSV');
%     zlabel('Normalised Magnitude');
% set(0,'CurrentFigure',3);
% %subplot(1,2,2);%colorbar;
%    % set(gca,'xTickLabel',trajectoryName{1:end-1});
%    % set(gca,'yTickLabel',{'\sigma_1','\sigma_2','\sigma_3','\sigma_4'});
%    
%     set(gca,'XTickLabel',{'T1','T2','T3','T4','T5'});
%     set(gca,'YTickLabel',{'s1','s2','s3','s4'});
%     xlabel('Trajectory');
%     ylabel('HSV');
%     zlabel('Normalised Magnitude');

%figure(3); 
%set(0,'CurrentFigure',2);
%bar3(resultMechHSV);

set(0,'CurrentFigure',4);
legend(trajectoryName{1:end-1},'Location','SouthEast'); grid on;axis tight; axis equal; a = axis;
am = max(a)/50; axis([a(1)-am a(2)+am a(3)-am a(4)+am]); xlabel('P_x position (m)','FontSize',14);ylabel('P_y position (m)','FontSize',14);

set(gca,'FontSize',14);
plot(0,0,'ko','MarkerSize',7,'LineWidth',2.5);plot(xD(1),xD(2),'ko','MarkerSize',7,'LineWidth',2.5);
set(gca,'FontSize',14);
%figure(4); 
set(0,'CurrentFigure',5);
p = plot(1:mechOpt.stateDim,cumsum(resultMech.hsv)./sum(resultMech.hsv),'k-o','LineWidth',2.0); hold on;
annotatePlotGroup(p);
p = plot(cumsum(ones(4,1)),redThreshold*ones(4,1),'k'); grid on;
annotatePlotGroup(p,'off');

legend(trajectoryName{1:end},'Location','SouthEast');

 %figure(5);
set(0,'CurrentFigure',6);

% colormap(jet(numTestTraj+1));
% bar(1:2,costFuncStore');
% subplot(2,1,1);bar(1:numTestTraj,hsvCostFuncStore(1:numTestTraj,:)); hold on;
% ylabel('Dimensionality D_W');
% 
% legend('Polynomial Basis','Fourier Basis','Location','NorthWest');
% set(gca,'xTick',1:numTestTraj+1,'xTickLabel',trajectoryName);
% 
% axis tight; grid on;
% ylabel('D_W');
% xlabel('Trajectory');

%subplot(2,1,2);
%colormap(jet(numTestTraj+2));
bh= bar(1:numTestTraj,costFuncStore(1:numTestTraj,:),'grouped','LineWidth',2.0);hold on;%,'EdgeColor',cols, 'LineWidth',2); hold on;
colormap(winter);
grid on;
% set(bh(1),'EdgeColor',cols(1,:));
% set(bh(2),'EdgeColor',cols(2,:));
% set(bh(3),'EdgeColor',cols(3,:));
% set(bh(4),'EdgeColor',cols(4,:));


axis tight;
ylabel('Cost J(\sigma _2)','FontSize',14);
set(gca,'xTick',1:numTestTraj+1,'xTickLabel',trajectoryName,'FontSize',14);
l = legend('Legendre Basis','Fourier Basis','Location','NorthWest');%,'FontSize',12);
set(l,'FontSize',14);
%haxes = plotyy(1:numTestTraj,hsvCostFuncStore(1:numTestTraj,:),1:numTestTraj,costFuncStore(1:numTestTraj,:));

%axes(haxes(2));

%legend('Polynomial Basis','Fourier Basis','Location','NorthWest');
axis tight; grid on;
%ylabel('D_W');
xlabel('Trajectory','FontSize',14);

set(0,'CurrentFigure',7);
%colormap(gray);
bar(cumsum(resultMech.hsv)./sum(resultMech.hsv),'grouped','FaceColor',0.7*[1 1 1],'LineWidth',2.5);

intrin = cumsum(resultMech.hsv)./sum(resultMech.hsv)
axis tight;
 a = axis;
 kT = sum(resultMechHSVMod(i,:)<result(i,j).threshold);
 %line([a(1); a(2)],[result(i,j).threshold ; result(i,j).threshold],'Color','k','LineWidth',2);
 line([a(1); a(2)],[ 0.9 ; 0.9],'Color','k','LineWidth',2);
        
grid on;
xlabel('HSV','FontSize',14);
ylabel('Normalised Magnitude','FontSize',14);
set(gca,'XTickLabel',{'s1','s2','s3','s4'},'FontSize',14);
%set(gca,'xTick',1:numTestTraj,'xTickLabel',trajectoryName,'FontSize',12);

%colormap(hsv(numTestTraj+1));
testT = 0:0.001:3.0;
%figure(6);
set(0,'CurrentFigure',10);
basisOpt{1}.tmax = testT(end);
plot(testT,basisFunc{1}(testT,basisOpt{1}),'LineWidth',2.0); grid on;
xlabel('Time (sec)','FontSize',14);
ylabel('Basis Functions \Psi_l(t)','FontSize',14);
set(gca,'FontSize',14);

%figure(7);
set(0,'CurrentFigure',9);
basisOpt{2}.tmax = testT(end);
plot(testT,basisFunc{2}(testT,basisOpt{2}),'LineWidth',2.0); grid on;
xlabel('Time (sec)','FontSize',14);
ylabel('Basis Functions \Psi_f(t)','FontSize',14);
set(gca,'FontSize',14);

set(0,'CurrentFigure',11);
subplot(2,2,1); set(gca,'FontSize',14);
subplot(2,2,2); set(gca,'FontSize',14);
subplot(2,2,3); set(gca,'FontSize',14);
subplot(2,2,4); set(gca,'FontSize',14);

set(0,'CurrentFigure',12);
set(gca,'FontSize',14);
subplot(2,2,1); set(gca,'FontSize',14);
subplot(2,2,2); set(gca,'FontSize',14);
subplot(2,2,3); set(gca,'FontSize',14);
subplot(2,2,4); set(gca,'FontSize',14);

figFolder = './Plots/SynergyPaper/TetheredMass2D_BasisComparisonTSDA/';

set(0,'CurrentFigure',7);figTitle = 'HSVIntrinsic';
print('-depsc2','-r800',strcat(figFolder,figTitle));

set(0,'CurrentFigure',2); figTitle = 'HSVsMultiTaskLegendre';
print('-depsc2','-r800',strcat(figFolder,figTitle));

set(0,'CurrentFigure',3); figTitle = 'HSVsMultiTaskFourier';
print('-depsc2','-r800',strcat(figFolder,figTitle));

set(0,'CurrentFigure',4); figTitle = 'CartesianTrajectories';
print('-depsc2','-r800',strcat(figFolder,figTitle));



set(0,'CurrentFigure',6); figTitle = 'HSV2CostComparison';
print('-depsc2','-r800',strcat(figFolder,figTitle));

set(0,'CurrentFigure',11); figTitle = 'Hinton_Legendre';
print('-depsc2','-r800',strcat(figFolder,figTitle));

set(0,'CurrentFigure',12); figTitle = 'Hinton_Fourier';
print('-depsc2','-r800',strcat(figFolder,figTitle));

set(0,'CurrentFigure',9); figTitle = 'LegendreBasis';
print('-depsc2','-r800',strcat(figFolder,figTitle));


set(0,'CurrentFigure',10); figTitle = 'FourierBasis';
print('-depsc2','-r800',strcat(figFolder,figTitle));


end