% 
% tspanRange = 1.0:0.5:3.0;        
% 
% cols = colormap(hsv(length(tspanRange)));
% for i = 1:length(tspanRange)
% 
%   %  xd = xD;
%     % Data for 0.3, -0.7 reaching
%     if(mod(tspanRange(i),1)==0)
%         load(sprintf('./Data/PassiveCompliantKinematicChainBasisInput/optimResult2_HSV2_tSpan_%d_target_%d_%d_down_cm10_order9',...
%             10*xd(1),10*xd(2)),...
%             'result','WOpt' ,'WIni', 'redThreshold','cm', 'tspan', 'mechSystem',...
%             'mechOpt', 'basisFunc', 'basisOpt');
%     else
%         load(sprintf('./Data/PassiveCompliantKinematicChainBasisInput/optimResult2_HSV2_tSpan_%d_5_target_%d_%d_down_cm10_order9',...
%             10*xd(1),10*xd(2)),...
%             'result','WOpt' ,'WIni', 'redThreshold','cm', 'tspan', 'mechSystem',...
%             'mechOpt', 'basisFunc', 'basisOpt');
%     end
%     [ result ] = plotMultiOptimalSolution( WOpt , WIni, redThreshold, cm, tspan, mechSystem, mechOpt, basisFunc, basisOpt,cols(i,:));
% end
% 
% figure(2);
% a = axis;
% axis(a*1.1);
% legend('P1','P2','P3','P4','P5','P6','P7','P8','P9','Location','NorthEast');
% figure(3);subplot(2,1,1);legend('P1','P2','P3','P4','P5','P6','P7','P8','P9','Location','NorthWest','Orientation','Horizontal');
% figure(4);subplot(2,1,1);legend('P1','P2','P3','P4','P5','P6','P7','P8','P9','Location','NorthEast','Orientation','Horizontal');
% 

% 
% figFolder = './Plots/PassiveCompliantKinematicChain_BasisInput/optim_2_';
% figure(2); figTitle = 'cartesianTrajectory';
% print('-depsc2','-r800',strcat(figFolder,figTitle));
% 
% figure(3); figTitle = 'posWrtTime';
% print('-depsc2','-r800',strcat(figFolder,figTitle));
% 
% figure(4); figTitle = 'velWrtTime';
% print('-depsc2','-r800',strcat(figFolder,figTitle));


xDRange = { 
    [0.6852,    0.2772]',...
    [0.6237,    0.2427]',...
    [0.5719,    0.1948]',...
    [0.5327,    0.1362]',...
    [0.5083,    0.0700]',...
    [0.5000,    0.0000]'
};

tDRange = 1.5:0.5:2.0;

ExitFlagStore = zeros(length(xDRange),length(tDRange));


cols = colormap(winter(length(tDRange)));
for i = 1:length(xDRange)

    xd = xDRange{i};
    for j = 1:length(tDRange)
    %j=i;
    
       % Data for 0.3, -0.7 reaching
         if(mod(tDRange(j),1) == 0)
            try
                %optimResult2_HSV2_tSpan_%d_target_%d_%d_down_cm10_order5',...
                load(sprintf('./Data/PassiveCompliantKinematicChainBasisInput/arc/MDReaching_tSpan_%d_target_%d_%d_down_order5.mat',...         
                floor(tDRange(j)),floor(10000*xd(1)),floor(10000*xd(2))),...
                'result','WOpt' ,'WIni', 'redThreshold','cm', 'tspan', 'mechSystem',...
                'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            catch
                fprintf('File does not exist : MDReaching_tSpan_%d_target_%d_%d_down_order5.mat\n',...
                floor(tDRange(j)),floor(10000*xd(1)),floor(10000*xd(2)));
                ExitFlag = 0;
            end
         else
             try
                 load(sprintf('./Data/PassiveCompliantKinematicChainBasisInput/arc/MDReaching_tSpan_%d_5_target_%d_%d_down_order5.mat',... 
                 floor(tDRange(j)),floor(10000*xd(1)),floor(10000*xd(2))),...
                 'result','WOpt' ,'WIni', 'redThreshold','cm', 'tspan', 'mechSystem',...
                 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
             catch
                fprintf('File does not exist : MDReaching_tSpan_%d_5_target_%d_%d_down_order5.mat\n',...
                floor(tDRange(j)),floor(10000*xd(1)),floor(10000*xd(2)));
                ExitFlag = 0;
           
             end
         end
         ExitFlagStore(i,j) = ExitFlag;
         if ExitFlag >0  
            [ result ] = plotMultiOptimalSolution( WOpt , WIni, redThreshold, cm, tspan, mechSystem, mechOpt, basisFunc, basisOpt,cols(j,:));

            
            fprintf('Accepted : %1.4f , %1.4f @ tspan %2.1f\n',xd(1),xd(2), tDRange(j));
         else
            fprintf('Rejected : %1.4f , %1.4f @ tspan %2.1f ExitFlag : %d \n',xd(1),xd(2), tDRange(j),ExitFlag);
         end
   end
end

disp(ExitFlagStore);