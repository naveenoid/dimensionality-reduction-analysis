% Load files

% Plot poly optim traj vs time for 1.5


% extract peak velocity for poly



basisOrderRange = 5;%:1:6;%[5,8];%5:1:8;%5:1:8;
tspanRange = 1.0;%:0.5:2.0;%2.5;%1:0.5:1.5;%2.0;%1:0.5:3;%8
endPointRange = [0.4 0.6:0.2:1.0];%0.8:0.2:1.0;%0.4:0.1:0.5%0.1:0.1:0.5;
result = struct();
%resultFourier = struct();
i = 0;j = 0; k = 0;

cols = colormap(hsv(length(endPointRange)+1));
%cols = colormap(hsv(length(basisOrderRange)+1));

tot = length(endPointRange+1);

for endPoint = endPointRange
   i = i+1;
    for basisOrder = basisOrderRange 
        j = j+1;
        for tspanMax = tspanRange
            k = k+1;
            
            fprintf('\n-----------------------------------------------------------\n');
            fprintf('POLYExtract : optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(endPoint),floor(endPoint));
            
            clearvars -except tspanRange tspanMax basisOrderRange basisOrder endPointRange endPoint resultPoly resultFourier i j k cols tot
            load(sprintf('./Data/TetheredMassPolyBasisInput/optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint)),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            %result(i,j,k,1).result = testOptimalSolutionPoly(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
            result = testOptimalSolutionPoly(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
%             
%             figure(j);
%             subplot(2,2,1);p1 = plot(result(2).t./max(result(2).t),result(2).x(:,1),'Color',cols(i,:),'LineWidth',2); hold on; title('Poly'); xlabel('t (sec)'); ylabel('x (m)'); axis tight;
%             annotatePlotGroup(p1);grid on;
%             subplot(2,2,2);p2 = plot(result(2).t./max(result(2).t),result(2).x(:,2),'Color',cols(i,:),'LineWidth',2); hold on; title('Poly'); xlabel('t (sec)'); ylabel('y (m)'); axis tight;
%             annotatePlotGroup(p2);grid on;
%             
%             figure(j+1);
%             subplot(2,2,1);p1 = plot(result(2).t./max(result(2).t),result(2).x(:,3),'Color',cols(i,:),'LineWidth',2); hold on; title('Poly'); xlabel('t (sec)'); ylabel('\dot{x} (m/sec)'); axis tight;
%             annotatePlotGroup(p1);grid on;
%             subplot(2,2,2);p2 = plot(result(2).t./max(result(2).t),result(2).x(:,4),'Color',cols(i,:),'LineWidth',2); hold on; title('Poly'); xlabel('t (sec)'); ylabel('\dot{y} (m/sec)'); axis tight;
%             annotatePlotGroup(p2);grid on;
%             
%             fprintf('FourierExtract : optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(endPoint),floor(endPoint));
%             fprintf('\n-----------------------------------------------------------\n');
%             
%             clearvars -except tspanRange tspanMax basisOrderRange basisOrder endPointRange endPoint resultPoly resultFourier i j k cols tot
%             load(sprintf('./Data/TetheredMassFourierBasisInput/optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint)),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
%             %result(i,j,k,2).result = testOptimalSolutionFourier(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
%             result = testOptimalSolutionFourier(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
%             
%             figure(j);
%             subplot(2,2,3);plot(result(2).t./max(result(2).t),result(2).x(:,1),'Color',cols(i,:),'LineWidth',2); hold on; title('Fourier'); xlabel('t (sec)'); ylabel('x (m)'); axis tight;
%             grid on;
%             subplot(2,2,4);plot(result(2).t./max(result(2).t),result(2).x(:,2),'Color',cols(i,:),'LineWidth',2); hold on; title('Fourier'); xlabel('t (sec)'); ylabel('y (m)'); axis tight;
%             grid on;
%             
%             figure(j+1);          
%             subplot(2,2,3);plot(result(2).t./max(result(2).t),result(2).x(:,3),'Color',cols(i,:),'LineWidth',2); hold on; title('Fourier'); xlabel('t (sec)'); ylabel('\dot{x} (m/sec)'); axis tight;
%             grid on;
%             subplot(2,2,4);plot(result(2).t./max(result(2).t),result(2).x(:,4),'Color',cols(i,:),'LineWidth',2); hold on; title('Fourier'); xlabel('t (sec)'); ylabel('\dot{y} (m/sec)'); axis tight;
%             grid on;
            
            figure(j);
            subplot(2,1,1);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,1),'Color',cols(i,:),'LineWidth',2); 
            hold on; title('Poly'); 
            xlabel('t (sec)'); ylabel('position x (m)'); axis tight;
            %if(endPoint == endPointRange(1)) 
                annotatePlotGroup(p);
            %else
            %    annotatePlotGroup(p,'off');
            %end
            grid on;
            %subplot(2,2,2);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,2),'Color',cols(i,:),'LineWidth',2); 
            hold on; %title('Poly'); 
            %xlabel('t (sec)'); ylabel('y (m)'); axis tight;
            %annotatePlotGroup(p);
            %if(endPoint == endPointRange(1))
                annotatePlotGroup(p,'off');
            %else
             %   annotatePlotGroup(p,'off');
            %end
            grid on;
            
            figure(j+1);
            subplot(2,1,1);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,3),'Color',cols(i,:),'LineWidth',2);
            hold on; title('Poly'); 
            xlabel('t (sec)'); ylabel('velocity dx/dt (m/sec)'); axis tight;
          %  if(endPoint == endPointRange(1)) 
                annotatePlotGroup(p);
           % else
           %     annotatePlotGroup(p,'off');
           % end
            %annotatePlotGroup(p1);
            grid on;
            %subplot(2,2,2);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,4),'Color',cols(i,:),'LineWidth',2);
            hold on; %title('Poly'); 
            xlabel('t (sec)'); ylabel('velocity dx/dt (m/sec)'); axis tight;
            %annotatePlotGroup(p2);
            %if(endPoint == endPointRange(1)) 
                annotatePlotGroup(p,'off');
            %else
            %    annotatePlotGroup(p,'off');
            %end
            grid on;
            
            fprintf('FourierExtract : optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(endPoint),floor(endPoint));
            fprintf('\n-----------------------------------------------------------\n');
            
            clearvars -except tspanRange tspanMax basisOrderRange basisOrder endPointRange endPoint resultPoly resultFourier i j k cols tot
            load(sprintf('./Data/TetheredMassFourierBasisInput/optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint)),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            %result(i,j,k,2).result = testOptimalSolutionFourier(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
            result = testOptimalSolutionFourier(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
            
            figure(j);
            subplot(2,1,2);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,1),'Color',cols(i,:),'LineWidth',2); hold on; title('Fourier'); %xlabel('t (sec)'); ylabel('x (m)'); axis tight;
            grid on;
            annotatePlotGroup(p,'off');
            xlabel('t (sec)'); ylabel('position x (m)'); axis tight;
            %subplot(2,2,4);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,2),'Color',cols(i,:),'LineWidth',2); hold on; title('Fourier'); %xlabel('t (sec)'); ylabel('y (m)'); axis tight;
            grid on;
            annotatePlotGroup(p,'off');
            xlabel('t (sec)'); ylabel('position x (m)'); axis tight;
            
            figure(j+1);          
            subplot(2,1,2);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,3),'Color',cols(i,:),'LineWidth',2); hold on; title('Fourier'); %xlabel('t (sec)'); ylabel('\dot{x} (m/sec)'); axis tight;
            grid on;
            annotatePlotGroup(p,'off');
            xlabel('t (sec)'); ylabel('velocity dx/dt (m/sec)'); axis tight;
            %subplot(2,2,4);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,4),'Color',cols(i,:),'LineWidth',2); hold on; title('Fourier'); %xlabel('t (sec)'); ylabel('\dot{y} (m/sec)'); axis tight;
            grid on;
            annotatePlotGroup(p,'off');
            xlabel('t (sec)'); ylabel('velocity dx/dt (m/sec)'); axis tight;

            figure(1);subplot(2,1,1);
            a = axis; axis([a(1) a(2) a(3) 1.25*a(4)]);
            legend('0.4','0.6','0.8','1.0','Location','North','Orientation','Horizontal'); grid on;
            figure(2);subplot(2,1,1);
            a = axis; axis([a(1) a(2) a(3) 1.35*a(4)]);
            legend('0.4','0.6','0.8','1.0','Location','North','Orientation','Horizontal'); grid on;

%             
        end
        k = 0;
    end
    j= 0;
    
    %figure(i);subplot(2,2,1);legend(int2str(basisOrderRange'));
%     figure(1);subplot(2,1,1);legend(int2str(10*endPointRange'),'Location','NorthWest'); grid on;
% 
%     %figure(tot+i);subplot(2,2,1);legend(int2str(basisOrderRange'));
%     figure(1+1);subplot(2,1,1);legend(int2str(10*endPointRange'),'Location','NorthWest'); grid on;
end



figFolder = './Plots/TetheredMassBasisInput/optim_1_';
figure(1); figTitle = 'position';
print('-depsc2','-r800',strcat(figFolder,figTitle));


figure(2); figTitle = 'velocity';
print('-depsc2','-r800',strcat(figFolder,figTitle));

% 
% pause;
% close all; clear all; 
% basisOrderRange = 5;%:1:6;%[5,8];%5:1:8;%5:1:8;
% tspanRange = 1.0:0.5:2.0;%2.5;%1:0.5:1.5;%2.0;%1:0.5:3;%8
% endPointRange = 1.0;%0.8:0.2:1.0;%0.4:0.1:0.5%0.1:0.1:0.5;
% result = struct();
% %resultFourier = struct();
% i = 0;j = 0; k = 0
% tot = length(endPointRange+1);
% 
% cols = colormap(hsv(length(tspanRange)+1));
% 
% for endPoint = endPointRange
%    i = i+1;
%     for basisOrder = basisOrderRange 
%         j = j+1;
%         for tspanMax = tspanRange
%             k = k+1;
%             
%             fprintf('\n-----------------------------------------------------------\n');
%             fprintf('POLYExtract : optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(endPoint),floor(endPoint));
%             
%             clearvars -except tspanRange tspanMax basisOrderRange basisOrder endPointRange endPoint resultPoly resultFourier i j k cols tot
%             load(sprintf('./Data/TetheredMassPolyBasisInput/optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint)),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
%             %result(i,j,k,1).result = testOptimalSolutionPoly(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
%             result = testOptimalSolutionPoly(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
%             
% %             figure(i);
% %             subplot(2,1,1);
% %             p1 = plot(result(2).t./max(result(2).t),result(2).x(:,1),'Color',cols(k,:),'LineWidth',2); hold on; title(sprintf('Poly : %d',endPoint)); xlabel('t (sec)'); ylabel('x (m)'); axis tight; grid on;
% %             annotatePlotGroup(p1,'off');
% %             
% %             %subplot(2,2,2);
% %             p2 = plot(result(2).t./max(result(2).t),result(2).x(:,2),'Color',cols(k,:),'LineWidth',2); hold on; title('Poly'); xlabel('t (sec)'); ylabel('y (m)'); axis tight; grid on;
% %             annotatePlotGroup(p2);
%             
%             figure(tot+i);
%             subplot(2,1,1);
%             p1 = plot(result(2).t./max(result(2).t),result(2).x(:,3),'Color',cols(k,:),'LineWidth',2); hold on; title(sprintf('Poly : %d',endPoint)); xlabel('t (sec)'); ylabel('\dot{x} (msec^-1)'); axis tight;  grid on;
%             annotatePlotGroup(p1,'off');
%             %subplot(2,2,2);
%             p2 = plot(result(2).t./max(result(2).t),result(2).x(:,4),'Color',cols(k,:),'LineWidth',2); hold on; title('Poly'); xlabel('t (sec)'); ylabel('\dot{y} (msec^-1)'); axis tight;  grid on;
%             annotatePlotGroup(p2);
%             
%             fprintf('FourierExtract : optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(endPoint),floor(endPoint));
%             fprintf('\n-----------------------------------------------------------\n');
%             
%             clearvars -except tspanRange tspanMax basisOrderRange basisOrder endPointRange endPoint resultPoly resultFourier i j k cols tot
%             load(sprintf('./Data/TetheredMassFourierBasisInput/optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint)),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
%             %result(i,j,k,2).result = testOptimalSolutionFourier(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
%             result = testOptimalSolutionFourier(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
%             
% %             figure(i);
% %             subplot(2,1,2);
% %             plot(result(2).t./max(result(2).t),result(2).x(:,1),'Color',cols(k,:),'LineWidth',2); hold on; title(sprintf('Fourier : %d',endPoint)); xlabel('t (sec)'); ylabel('x (m)'); axis tight;  grid on;
% %             %subplot(2,2,4);
% %             plot(result(2).t./max(result(2).t),result(2).x(:,2),'Color',cols(k,:),'LineWidth',2); hold on; title('Fourier'); xlabel('t (sec)'); ylabel('y (m)'); axis tight;  grid on;
%             
%             figure(tot+i);            
%             subplot(2,1,2);
%             plot(result(2).t./max(result(2).t),result(2).x(:,3),'Color',cols(k,:),'LineWidth',2); hold on; title(sprintf('Fourier : %d',endPoint)); xlabel('t (sec)'); ylabel('\dot{x} (msec^-1)'); axis tight;  grid on;
%             %subplot(2,2,4);
%             plot(result(2).t./max(result(2).t),result(2).x(:,4),'Color',cols(k,:),'LineWidth',2); hold on; title('Fourier'); xlabel('t (sec)'); ylabel('\dot{y} (msec^-1)'); axis tight;  grid on;
% %             
%         end
%         k = 0;
%     end
%     j= 0;
%     
%     %figure(i);subplot(2,2,1);legend(int2str(basisOrderRange'));
%     figure(i);subplot(2,1,1);legend('1','1.5','2','2.5');%legend(int2str(tspanRange'));
% 
%     %figure(tot+i);subplot(2,2,1);legend(int2str(basisOrderRange'));
%     figure(tot+i);subplot(2,1,1);legend('1','1.5','2','2.5');%legend(int2str(tspanRange'));
% end
% 
% 
% %save('./Data/TetheredMassOptimResult/optimResult','result','basisOrderRange','tspanRange','endPointRange');