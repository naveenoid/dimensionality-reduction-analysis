%Nice Lines : 
xDRange = {[0.6,0.0]',[0.5,0.1]',[0.6,0.4]',[0.7,0.1]',[0.7,0.2]', [0.7,0.3]'};%{[0.7,0.2]',[0.7,0.3]',[0.6,0.0]',[0.6,0.4]',[0.6,0.2]',[0.5,0.1]'};%,[0.5,0.3]',[0.4,0.0]',[0.4,0.6]',[0.2,0.4]'};
tDRange = {2.0, 2.0, 2.0, 1.5, 1.5,1.5};%{2.0,  1.5, 2.0, 2.0,  1.5,2.5,2.5};

%5xDRange = {[0.6,0.4]',[0.7,0.1]',[0.7,0.3]'};%,[0.4,0.2]',[0.4,0.2]'};%{[0.6,0.0]',[0.7,0.3]',[0.6,0.4]',[0.5,0.3]',[0.5,0.1]'};

%xDRange = {[0.6,0.4]',[0.4,0.6]'};%,[0.4,0.6]',[0.4,0.2]',[0.2,0.4]'};
%tDRange = {2.0,2.0};%,2.0,2.0,2.5};


compWith = 2;%1 - Acceleration, 2 - Jerk

% 
% 
% if(comp == 1)
%     minCompFunc = @(t,xf,td)(1.5*xf.* (2*(t/td).^2 - (4/3)*(t/td).^3));
% else
%     minCompFunc = @(t,xf,td)( xf*(10*(t/td).^3 - 15*(t/td).^4 + 6*(t/td).^5 ) );
% end

%%%%Correct this
if(compWith == 1)
    minCompFunc = @(t,xf,xi,td)( xi+(xf-xi)*1.5*(2*(t/td).^2 - (4/3)*(t/td).^3));
else
    minCompFunc = @(t,xf,xi,td)( xi+(xf-xi)*(10*(t/td).^3 - 15*(t/td).^4 + 6*(t/td).^5 ) );
end
cols = colormap(winter(3));
cols = [cols;colormap(autumn(3+1))];
for i = 1:length(xDRange)

    xd = xDRange{i};
    %for j = 1:length(tDRange)
    j=i;
        % Data for 0.3, -0.7 reaching
         if(mod(tDRange{j},1) == 0)
            try
                %optimResult2_HSV2_tSpan_%d_target_%d_%d_down_cm10_order5',...
                load(sprintf('./Data/PassiveCompliantKinematicChainBasisInput/MDReaching_tSpan_%d_target_%d_%d_down_order5.mat',...         
                floor(tDRange{j}),10*xd(1),10*xd(2)),...
                'result','WOpt' ,'WIni', 'redThreshold','cm', 'tspan', 'mechSystem',...
                'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            catch
                fprintf('File does not exist : optimResult2_HSV2_tSpan_%d_target_%d_%d_down_cm10_order5',...
                floor(tDRange{j}),10*xd(1),10*xd(2));
                ExitFlag = 0; %colChoice = 1;
            end
         else
             try
                 load(sprintf('./Data/PassiveCompliantKinematicChainBasisInput/MDReaching_tSpan_%d_5_target_%d_%d_down_order5.mat',... 
                 floor(tDRange{j}),10*xd(1),10*xd(2)),...
                 'result','WOpt' ,'WIni', 'redThreshold','cm', 'tspan', 'mechSystem',...
                 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
             catch
                fprintf('File does not exist : optimResult2_HSV2_tSpan_%d_target_%d_%d_down_cm10_order5',...
                floor(tDRange{j}),10*xd(1),10*xd(2)); %colChoice = 2;
                ExitFlag = 0;
            end
         end

         if ExitFlag ==2
            [ result ] = plotMultiOptimalSolution( WOpt , WIni, redThreshold, cm, tspan, mechSystem, mechOpt, basisFunc, basisOpt,cols(i,:));
            
                figure(2);
                t = 0:0.001:tDRange{j};
                subplot(2,1,1);
                y1 = minCompFunc(t,xd(1),0.7846,tDRange{j});
                %y1 = minCompFunc(t,result(2).cartTraj(end,1),0.7846,tDRange{j});
                hold on; p = plot(t./tDRange{j},y1,'k--','LineWidth',2.0);
                
                if(i < length(xDRange))
                     annotatePlotGroup(p,'off');
                else
                     annotatePlotGroup(p);
                end
                subplot(2,1,2);
                y2 = minCompFunc(t,xd(2),0.0,tDRange{j});
                %y2 = minCompFunc(t,result(2).cartTraj(end,2),0.0,tDRange{j});
                hold on;p =  plot(t./tDRange{j},y2,'k--','LineWidth',2.0);
                
                if(i < length(xDRange))
                     annotatePlotGroup(p,'off');
                else
                     annotatePlotGroup(p);
                end
            figure(3);
                y1d = diff(y1)./diff(t);
                y2d = diff(y2)./diff(t);
                
                subplot(2,1,1); 
                hold on; p = plot(t(1:end-1)./tDRange{j}, y1d,'k--','LineWidth',2.0);
                if(i < length(xDRange))
                     annotatePlotGroup(p,'off');
                else
                     annotatePlotGroup(p);
                end
                subplot(2,1,2); 
                hold on; p = plot(t(1:end-1)./tDRange{j}, y2d,'k--','LineWidth',2.0);
                if(i < length(xDRange))
                     annotatePlotGroup(p,'off');
                else
                     annotatePlotGroup(p);
                end
                
                fprintf('Accepted : %d , %d @ tspan %2.1f\n',10*xd(1),10*xd(2), tDRange{j});
            
         else axis equal;
            fprintf('Rejected : %d , %d @ tspan %2.1f ExitFlag : %d \n',10*xd(1),10*xd(2), tDRange{j},ExitFlag);
         end
   % end
end
% 
% figure(2);
% a = axis;
% axis(a*1.1);
% legend('P1','P2','P3','P4','P5','P6','P7','P8','P9','Location','NorthEast');
% figure(3);subplot(2,1,1);legend('P1','P2','P3','P4','P5','P6','P7','P8','P9','Location','NorthWest','Orientation','Horizontal');
% figure(4);subplot(2,1,1);legend('P1','P2','P3','P4','P5','P6','P7','P8','P9','Location','NorthEast','Orientation','Horizontal');
% 

figure(1);
l = legend('2.0s','2.0s','2.0s','1.5s', '1.5s','1.5s','location','NorthWest');
set(l,'FontSize',14);
set(gca,'FontSize',14);
xlabel('P_x position (m)','FontSize',14);
ylabel('P_y position (m)','FontSize',14);

axis equal;a = axis;axis([0.45 0.85 -0.05 0.45]); 
plot(0.7846,0.0,'ok','LineWidth',2.0);

% axis([a(1)-0.05 a(3)+0.05 a(3)-0.05 a(4)+0.05]);



figure(2);
subplot(2,1,1);
if(compWith == 1)
    minFuncName = 'MA';
else
    minFuncName = 'MJ';
end


l = legend('(0.6,0.0)','(0.5,0.1)','(0.6,0.4)','(0.7,0.1)','(0.7,0.2)', '(0.7,0.3)',minFuncName,'location','SouthWest');
set(l,'FontSize',14);
set(gca,'FontSize',14);
subplot(2,1,2);
set(gca,'FontSize',14);
%xDRange = {[0.6,0.0]',[0.5,0.1]',[0.6,0.4]',[0.7,0.2]', [0.7,0.3]'};%{[0.7,0.2]',[0.7,0.3]',[0.6,0.0]',[0.6,0.4]',[0.6,0.2]',[0.5,0.1]'};%,[0.5,0.3]',[0.4,0.0]',[0.4,0.6]',[0.2,0.4]'};
%tDRange = {2.0, 2.0, 2.0, 1.5, 1.5,1.5};%{2.0,  1.5, 2.0, 2.0,  1.5,2.5,2.5};


figure(3);
subplot(2,1,1); set(gca,'FontSize',14);
subplot(2,1,2); set(gca,'FontSize',14);


if(compWith == 1)
    figFolder = './Plots/PassiveCompliantKinematicChainBasisInput/MD_PassiveKinematicChain_MinAccl_';
else
    figFolder = './Plots/PassiveCompliantKinematicChainBasisInput/MD_PassiveKinematicChain_MinJerk_';
end

figure(1); figTitle = 'cartesianTrajectory';
print('-depsc2','-r800',strcat(figFolder,figTitle));

figure(2); figTitle = 'position';
print('-depsc2','-r800',strcat(figFolder,figTitle));

figure(3); figTitle = 'velocity';
print('-depsc2','-r800',strcat(figFolder,figTitle));
