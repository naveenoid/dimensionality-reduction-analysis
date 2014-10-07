% Load files

% Plot poly optim traj vs time for 1.5


% extract peak velocity for poly



basisOrderRange = 1;%1:2;%4:5;%:1:6;%[5,8];%5:1:8;%5:1:8;

legendreBasisRange = [4,5];
fourierBasisRange = [4,5];
tspanRange = 0.8:0.2:1.2;%10:2.5:20;%5.0:0.5:7;%:0.5:2.0;%2.5;%1:0.5:1.5;%2.0;%1:0.5:3;%8
endPointRange = 0.5;%[0.4 0.6:0.2:1.0];%0.8:0.2:1.0;%0.4:0.1:0.5%0.1:0.1:0.5;
result = struct();
%resultFourier = struct();
i = 0;j = 0; k = 0;

WPoly = cell(2);
WFourier = cell(2);

%cols = colormap(hsv(length(endPointRange)+1));
%cols = colormap(hsv(length(basisOrderRange)+1));
cols = colormap(winter(length(tspanRange)));

%minCompDiffPoly = zeros(length(tspanRange));
%minCompDiffFourier = zeros(length(tspanRange));

comp = 1; % 1 - min accln, 2 - min jerk

if(comp == 1)
                 minCompFunc = @(t,xf,td)(1.5*xf.* (2*(t/td).^2 - (4/3)*(t/td).^3));
            else
             minCompFunc = @(t,xf,td)( xf*(10*(t/td).^3 - 15*(t/td).^4 + 6*(t/td).^5 ) );
end
            
 minCompFuncMA = @(t,xf,td)(1.5*xf.* (2*(t/td).^2 - (4/3)*(t/td).^3));
 minCompFuncMJ = @(t,xf,td)( xf*(10*(t/td).^3 - 15*(t/td).^4 + 6*(t/td).^5 ) );

tot = length(endPointRange+1);
i = 0;
for endPoint = endPointRange
   %i = i+1;j = 0;
   j = j+ 1; i = 0;
    for bo= basisOrderRange 
       i = i+1;k=0;       
       wPolyStore= zeros(2+2*legendreBasisRange(bo),length(tspanRange));
       wFourierStore = zeros(2+4*fourierBasisRange(bo),length(tspanRange));
        for tspanMax = tspanRange
            k = k+1;
            
            
            
            fprintf('\n-----------------------------------------------------------\n');
            %polyBasisOrder = basisOrder+2;
            basisOrder = legendreBasisRange(bo);
            fprintf('i:%d, j:%d, k:%d\n',i,j,k);
            
            fprintf('POLYExtract : MDCSigmoidStart_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),...
                floor(basisOrder),floor(endPoint),floor(endPoint));
            
            clearvars -except tspanRange tspanMax basisOrderRange basisOrder endPointRange endPoint resultPoly...
                resultFourier i j k cols tot legendreBasisRange fourierBasisRange basisOrder bo wPolyStore...
                wFourierStore wFourierStore WPoly WFourier minCompFunc comp minCompFuncMA minCompFuncMJ
            load(sprintf('./Data/SynergyPaper/TetheredMass2D_LegendreBasisMDC/MDC_SigmoidStart_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint)),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            
             wPolyStore(:,k) = WOpt; 
            %result(i,j,k,1).result = testOptimalSolutionPoly(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
            result = testOptimalSolutionPoly(WOpt, WIni, redThreshold,  tspan,...
                mechSystem, mechOpt, basisFunc,basisOpt);

            
            %figure(2*(j-1)+1);
            %
            figure(j);
            subplot(2,1,1);
           % p = plot(result(2).t./max(result(2).t),result(2).x(:,1),'Color',cols(k,:),'LineWidth',2); 
            p = plot(result(2).t,result(2).x(:,1),'Color',cols(k,:),'LineWidth',2); 
            hold on; title(sprintf('Legendre Basis Order : %d', legendreBasisRange(i)),'FontSize',14); 
            xlabel('Time t (sec)','FontSize',14); ylabel('Position \phi (m)','FontSize',14); axis tight;
            %if(endPoint == endPointRange(1)) 
                annotatePlotGroup(p);
            %else
            %    annotatePlotGroup(p,'off');
            %end
            grid on;
      
            p = plot(result(2).t,result(2).x(:,2),'Color',cols(k,:),'LineWidth',2);
            hold on; %title('Poly'); 
            
            annotatePlotGroup(p,'off');
            %else
            p = plot(result(2).t,minCompFuncMJ(result(2).t,result(2).x(end,1),tspanMax),'k--','LineWidth',2);

            
            if(k == length(tspanRange))
                annotatePlotGroup(p);
            else
                annotatePlotGroup(p,'off');
            end
            grid on;
            

            figure(j+1);

            subplot(2,1,1);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,3),'Color',cols(k,:),'LineWidth',2);
         
            hold on; title(sprintf('Legendre Basis Order : %d', legendreBasisRange(i)),'FontSize',14); 
            xlabel('Normalised Time t (sec)','FontSize',14); ylabel('Velocity d\phi/dt (m/sec)','FontSize',14); axis tight;

            annotatePlotGroup(p);
            
            %%minjerk
            
            posF = minCompFuncMJ(result(2).t,result(2).x(end,1),tspanMax);
            p = plot(result(2).t(1:end-1)./max(result(2).t),diff(posF)./diff(result(2).t),'k--','LineWidth',2);
            
            if(k == length(tspanRange))
                annotatePlotGroup(p);
            else
                annotatePlotGroup(p,'off');
            end
            

            grid on;

            p = plot(result(2).t./max(result(2).t),result(2).x(:,4),'Color',cols(k,:),'LineWidth',2);

            annotatePlotGroup(p,'off');
            xlabel('Normalised Time t/t_d (sec)','FontSize',14); ylabel('Velocity d\phi/dt (m/sec)','FontSize',14); axis tight;

            grid on;
            basisOrder = fourierBasisRange(bo);
            
            fprintf('FourierExtract : optimSigmoid_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(endPoint),floor(endPoint));
            fprintf('\n-----------------------------------------------------------\n');
            
            clearvars -except tspanRange tspanMax basisOrderRange basisOrder endPointRange endPoint...
                resultPoly resultFourier i j k cols tot  legendreBasisRange fourierBasisRange basisOrder bo...
                wPolyStore wFourierStore WPoly WFourier minCompFunc comp  minCompFuncMA minCompFuncMJ
            
            %load(sprintf('./Data/TetheredMassFourierBasisInput/optimSigmoidStart_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint)),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            load(sprintf('./Data/SynergyPaper/TetheredMass2D_FourierBasisMDC/optimSigmoidStart_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint)),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            wFourierStore(:,k) = WOpt;
            %result(i,j,k,2).result = testOptimalSolutionFourier(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
            result = testOptimalSolutionFourier(WOpt, WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
            
            figure(j);
            subplot(2,1,2);
            %p = plot(result(2).t./max(result(2).t),result(2).x(:,1),'Color',cols(k,:),'LineWidth',2); hold on; title('Fourier'); %xlabel('t (sec)'); ylabel('x (m)'); axis tight;
            p = plot(result(2).t,result(2).x(:,1),'Color',cols(k,:),'LineWidth',2); hold on; 
            title(sprintf('Fourier Basis Order : %d', fourierBasisRange(i)),'FontSize',14); %xlabel('t (sec)'); ylabel('x (m)'); axis tight;
            grid on;
            annotatePlotGroup(p);
            xlabel('Time t (sec)','FontSize',14); ylabel('Position \phi (m)','FontSize',14); axis tight;
            %subplot(2,2,4);
            %p = plot(result(2).t./max(result(2).t),result(2).x(:,2),'Color',cols(k,:),'LineWidth',2); hold on; title('Fourier'); %xlabel('t (sec)'); ylabel('y (m)'); axis tight;
            p = plot(result(2).t,result(2).x(:,2),'Color',cols(k,:),'LineWidth',2); hold on; 
            title(sprintf('Fourier Basis Order : %d', fourierBasisRange(i)),'FontSize',14); %xlabel('t (sec)'); ylabel('y (m)'); axis tight;
            grid on;
            annotatePlotGroup(p,'off');
            xlabel('Time t (sec)','FontSize',14); ylabel('Position \phi (m)','FontSize',14); axis tight;
            
            %% min jerk comparison
            hold on;
            p = plot(result(2).t,minCompFuncMA(result(2).t,result(2).x(end,1),tspanMax),'k--','LineWidth',2);
            annotatePlotGroup(p);
            
        
            figure(j+1);          
            subplot(2,1,2);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,3),'Color',cols(k,:),'LineWidth',2); 
            hold on; title(sprintf('Fourier Basis Order : %d', fourierBasisRange(i)),'FontSize',14); %xlabel('t (sec)'); ylabel('\dot{x} (m/sec)'); axis tight;
            grid on;
            annotatePlotGroup(p);
            xlabel('Normalised Time t/t_d (sec)','FontSize',14); ylabel('Velocity d\phi/dt (m/sec)','FontSize',14); axis tight;
            %subplot(2,2,4);
            p = plot(result(2).t./max(result(2).t),result(2).x(:,4),'Color',cols(k,:),'LineWidth',2); hold on; 
            title(sprintf('Fourier Basis Order : %d', fourierBasisRange(i)),'FontSize',14); %xlabel('t (sec)'); ylabel('\dot{y} (m/sec)'); axis tight;
            grid on;
            annotatePlotGroup(p,'off');
            xlabel('Normalised Time t/t_d (sec)','FontSize',14); ylabel('Velocity d\phi/dt (m/sec)','FontSize',14); axis tight;

            
            %%minjerk comparison
            posF = minCompFuncMA(result(2).t,result(2).x(end,1),tspanMax);
            p = plot(result(2).t(1:end-1)./max(result(2).t),diff(posF)./diff(result(2).t),'k--','LineWidth',2);
            annotatePlotGroup(p);
            
            
        end
        WPoly{i} = wPolyStore;
        WFourier{i} = wFourierStore;
    end
end

if(comp == 1)
    MinCompText = 'MA';
else
    MinCompText = 'MJ';
end
figure(1);
subplot(2,1,1);
% a = axis; axis([a(1) a(2) a(3) 1.25*a(4)]);
l = legend('0.8s ','1.0s ','1.2s ','MJ','Location','East'); grid on;% ,'Orientation','Horizontal'); grid on;
set(l,'FontSize',14);
xlabel('Time t (sec)','FontSize',14); ylabel('Position \phi (m)','FontSize',14);
set(gca,'FontSize',14);

subplot(2,1,2);
% a = axis; axis([a(1) a(2) a(3) 1.25*a(4)]);
%             l=legend('0.8s ','1.0s ','1.2s ','MJ','Location','East'); grid on;% ,'Orientation','Horizontal'); grid on;
%             set(l,'FontSize',14);
l = legend('0.8s ','1.0s ','1.2s ','MA','Location','East'); grid on;% ,'Orientation','Horizontal'); grid on;
set(l,'FontSize',14);
xlabel('Time t (sec)','FontSize',14); ylabel('Position \phi (m)','FontSize',14);
set(gca,'FontSize',14);

figure(2);subplot(2,1,1);
% a = axis; axis([a(1) a(2) a(3) 1.35*a(4)]);
a = axis; axis([a(1) a(2) a(3) 1.05*a(4)]);
l = legend('0.8s ','1.0s ','1.2s ','MJ','Location','South'); grid on;%,'Orientation','Horizontal'); grid on;
set(l,'FontSize',14);
xlabel('Normalised Time t/t_d (sec)','FontSize',14); ylabel('Velocity d\phi/dt (m/sec)','FontSize',14); 
set(gca,'FontSize',14);
subplot(2,1,2);
%             % a = axis; axis([a(1) a(2) a(3) 1.35*a(4)]);
%             a = axis; axis([a(1) a(2) a(3) 1.05*a(4)]);
%             l = legend('0.8s ','1.0s ','1.2s ','MJ','Location','South'); grid on;%,'Orientation','Horizontal'); grid on;
%             set(l,'FontSize',14);
l = legend('0.8s ','1.0s ','1.2s ','MA','Location','South'); grid on;%,'Orientation','Horizontal'); grid on;
set(l,'FontSize',14);
xlabel('Normalised Time t/t_d (sec)','FontSize',14); ylabel('Velocity d\phi/dt (m/sec)','FontSize',14); 
set(gca,'FontSize',14);

%  pause;

if (length(basisOrderRange)>1)

    figure(3);
    subplot(2,1,1);
    wp = WPoly{1};
    wpMax = max(wp);
    wp = wp./repmat(wpMax,size(wp,1),1);
    bar(wp(1:end/2,:,:)); axis tight;grid on;
    title(sprintf('Order : %d', legendreBasisRange(1)));
    subplot(2,1,2);
    wp = WPoly{2};
    wpMax = max(wp);
    wp = wp./repmat(wpMax,size(wp,1),1);
    bar(wp(1:end/2,:,:)); axis tight; grid on;
    title(sprintf('Order : %d', legendreBasisRange(2)));


    figure(4);
    subplot(2,1,1);
    wf = WFourier{1};
    wfMax = max(wf);
    wf = wf./repmat(wfMax,size(wf,1),1);
    bar(wf(1:end/2,:,:)); axis tight;grid on;
    title(sprintf('Order : %d', fourierBasisRange(1)));
    subplot(2,1,2);
    wf = WFourier{2};
    wfMax = max(wf);
    wf = wf./repmat(wfMax,size(wf,1),1);
    bar(wf(1:end/2,:,:)); axis tight; grid on;
    title(sprintf('Order : %d', fourierBasisRange(2)));

else
    
    
    figure(3);
    subplot(2,1,1);
%    subplot(2,1,1);
    wp = WPoly{1};
   % wpMax = max(wp);
   % wp = wp./repmat(wpMax,size(wp,1),1);
    bar(wp(1:end/2,:,:)); axis tight;grid on;
    colormap(winter);
    title(sprintf('Legendre Basis Order : %d', legendreBasisRange(1)),'FontSize',14);
    %legend('t_d = 0.8','t_d = 1.0','t_d = 1.2');
    xlabel('Weight w_i','FontSize',14);%set(l,'FontSize',14);
    ylabel('Magnitude','FontSize',14);%set(l,'FontSize',14);
    a = axis; axis([a(1) a(2) 1.05*a(3) 1.05*a(4)]);
    subplot(2,1,2);
       % subplot(2,1,1);
    wf = WFourier{1};
    %wfMax = max(wf);
    %wf = wf./repmat(wfMax,size(wf,1),1);
    bar(wf(1:end/2,:,:)); axis tight;grid on;
    colormap(winter);
    title(sprintf('Fourier Basis, Order : %d', fourierBasisRange(1)),'FontSize',14);
    l = legend('0.8s ','1.0s ','1.2s ','Location','SouthEast','Orientation','Horizontal');
    set(l,'FontSize',14);
    xlabel('Weight w_i','FontSize',14);%set(l,'FontSize',14);
    ylabel('Magnitude','FontSize',14);%set(l,'FontSize',14);
    a = axis; axis([a(1) a(2) 1.05*a(3) 1.05*a(4)]);


end

if( comp==1)
    figFolder =  './Plots/SynergyPaper/TetheredMass2D_MDCAnalysis/MDTetheredMass_MinAccl_';
else
    figFolder =  './Plots/SynergyPaper/TetheredMass2D_MDCAnalysis/MDTetheredMass_MinJerk_';
end
figure(1); figTitle = 'Position';set(gca,'FontSize',14);
print('-depsc2','-r800',strcat(figFolder,figTitle));


figure(2); figTitle = 'Velocity';set(gca,'FontSize',14);
print('-depsc2','-r800',strcat(figFolder,figTitle));

figure(3); figTitle = 'multiTimeWeightChangePolyFourier';set(gca,'FontSize',14);
print('-depsc2','-r800',strcat(figFolder,figTitle));
% 
