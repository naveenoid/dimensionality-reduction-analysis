%% Script to test How the DMP basis system works with the spring mass chain mech system.

chainLengthRange = 2;%5:5:10;

tempVar=0;

normhsvStore = zeros(length(chainLengthRange),chainLengthRange(end));
for chainLength  = chainLengthRange

    clearvars -except chainLength chainLengthRange normhsvStore tempVar

    
    numTestTraj = 4;%4;%5;%4;%4;
    cols = colormap(hsv(numTestTraj+1));
    
    basisOpt.N= 20;
   
    hsvStore = zeros(numTestTraj,chainLength*2);
    WHatStore = zeros(4,basisOpt.N);
    
    for i =1:numTestTraj
    
        tempVar = tempVar+1;
        clearvars -except tempVar chainLength hsvStore WHatStore result cols i
        
            %     mechOpt = robotData();
    %     xi = mechOpt.initialCond;
    %  tspan = [0;1];%[0:0.001:10];%train.tspan;
    %     mechSystem = @planar2dofArm;
    %     mechOpt.inputDim = 2;
    %     mechOpt.stateDim = 4;
    %     mechOpt.outputDim = 2;
    %     mechOpt.outputs = [1,2];
        tspanMax = 1;

        mechOpt = init1DChain(chainLength);%init2DSpringyDampedMass(2);%robotData();
        mechOpt.initialCond = zeros(size(mechOpt.A,1),1);
        xi = mechOpt.initialCond;
        tspan = [0;tspanMax];%[0:0.001:10];%train.tspan;
        mechSystem = @sysLTIForced;
        mechOpt.inputDim = size(mechOpt.B,2);
        mechOpt.stateDim = size(mechOpt.A,1);
        mechOpt.outputDim = size(mechOpt.C,1);
        mechOpt.outputs = [chainLength];

     %   cm = 1.0;%0.945;%1.75;
        redThreshold = 0.985;%0.994;%0.985;
        mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);


        trajectoryName = {'T1','T2','T3','T4','\Delta_i'};%{'straightLine', 'curvedPoly', 'S-Curve','OvershootCurve','intrinsic'};

       % fprintf('Learning Training Trajectory\n');
            % trainingData 
        % xStart = [0;0]; 
        xD = 1.00;%[0.5,-0.5]';%[0.5;-0.6];%[0.5;-0.5];

        desiredPos = xD;
       % xStart = mechOpt.initialCond(1:2);

        result= struct();
        tTrain =linspace(tspan(1),tspan(2),500)';%(tspan(1):0.05:tspan(2));

       % uD = zeros(2,length(tTrain));

         %   tic;
       % fprintf('Learning Training Trajectory\n');
            % trainingData 
        %xStart = [0]; 
       % xD = [0.5];
        %numPoints  = 1;

        basisOpt.N= 20;%50;%12; 
       basisOpt.rescale = 100;% 10;%10000;
        %basisOpt.order = basisOpt.N;
       % load(sprintf('./Data/DMPBasis/psiStar_N%d_td100',basisOpt.N ),'PsiStar','tOut');
        load(sprintf('./Data/DMPPaper/DMPBasis/psiStar_N%d',basisOpt.N ),'PsiStar','tOut');
      %  PsiStarTemp = PsiStar;
      %  PsiStarTemp(2:basisOpt.N+2,:) = PsiStar(1:basisOpt.N+1,:);
      % PsiStarTemp(1,:) = PsiStar(basisOpt.N+2,:);
      %  PsiStar = PsiStarTemp;

        PsiStar(1:basisOpt.N,:) = basisOpt.rescale*PsiStar(1:basisOpt.N,:);
        basisOpt.psiStar=PsiStar;
        basisOpt.tDes = tOut;

        tplot = tOut;%linspace(0,1,100);
        if(tempVar==1)
            figure(1);colormap(winter);
            %plot(tOut,PsiStar(1:basisOpt.N,:)','LineWidth',2);
            plot(tplot,dmpDataSys(tplot,basisOpt)','LineWidth',2);
            xlabel('Time t(sec)','FontSize',14);ylabel('Magnitude','FontSize',14);
            grid on;axis tight;
        %     
        %     figure(2);
        %     plot(tOut, PsiStar(basisOpt.N+1:basisOpt.N+2,:)','LineWidth',2);
        %     xlabel('Time t(sec)','FontSize',14);ylabel('Magnitude','FontSize',14);
        %     grid on;axis tight;
        %     
            %PsiStar(1:basisOpt.N,:) = PsiStar(1:basisOpt.N,:).*basisOpt.rescale;
        end

        %polyBasis setup;
        basisFunc= @dmpDataSys;
        basisLearning = @dmpBasisWeightLearning;

       % basisOpt.alphaZ = 25;
     %   basisOpt.N = 10;
      %  inputDim = basisOpt.N+2;
       % cols = winter(length(trajectoryName)); 

       
       % desiredPos = desiredPos+0.5;
       % clear x WHat t 
       % silentFigure(1);
        hold on;%+ 5*(j-1));
        %trainingFunc = generateBenchmarkTrajectories(xD, tspan,i, mechOpt,cols(i,:));
                                                             %     xStart, xEnd, tspan,trajectoryType, opt,cols 
        [trainingFunc, initCond] = generateBenchmarkTrajectoriesCartesian_SMC(0,desiredPos, tspan,i, mechOpt,cols(i,:));
        mechOpt.initialCond = initCond;xi = mechOpt.initialCond;

       
     %   result.uD = uD;
     % ( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
     % [WHat, basisOpt] = dmpLearning(tTrain, mechOpt.inputDim, trainingFunc,'nosmooth', basisOpt) ;%tTrain,uD,[],[],basisFunc{j},mechOpt,basisOpt{j});
                                   %  trainingTSpan, PsiStar, inputu,smoothening, DMPopt
     [WHat, basisOpt] = dmpPInvLearn(tTrain,PsiStar,trainingFunc,'nosmooth',basisOpt);
%      if(i == 4)
%          WHat = ones(size(WHat));
%      end
     
       % WHat2 = cumsum(1:length(WHat(1,:)));
        
        %basisOpt.psiStar = PsiStar;
        basisOpt.tDes = linspace(0,1,1000);%tOut;
        mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
        %inputDim = basisOpt.order;
        
        WHatStore(i,:) = WHat(1,1:basisOpt.N);%reshape(WHat',numel(WHat),1);
        mechBasisOpt = mechOpt;
        mechBasisOpt.inputDim = basisOpt.N+2;%basisOpt.order+1;
        %uFunc = @(t)WHat*basisFunc(t,basisOpt);
        
        figure(4); 
        subplot(2,2,i);
        bar(WHat(1,1:basisOpt.N+2),'grouped','FaceColor',cols(i,:),'LineWidth',1.0); axis tight; grid on;
        fprintf('\nWHat : \n');
        disp(WHat(1,1:5));
        
%         
%         
%         %% pure mech system
%         
%         syss = @(t,x)mechSystem(t,x,trainingFunc(t)',mechOpt);
% 
%         %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
%         %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
%         vopt = odeset('AbsTol',1e-3, 'RelTol',5e-4);
%         %
%         tspanFull = linspace(tspan(1),tspan(2),1000);
%         fprintf('Simulating time for simple step input ');
% 
%         tic;
%         [ts, xs] = ode15s(syss,tspanFull,xi,vopt);
%         toc
%         ys = mechOpt.C*xs';
%         
%         figure(8);
%         plot(ts,ys,'Color',cols(i,:)); axis tight; grid on;hold on;


       % sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

    %    [basisOpt] = lwrBatch(trainingTSpan, DMPdim, inputu,'nosmooth',basisOpt);
    %    basisOpt

        mechBasisOpt = mechOpt;
        mechBasisOpt.inputDim = basisOpt.N+2;
        %uFunc = @(t)WHat*basisFunc(t,basisOpt);


        sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

        %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
        %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
        vopt = odeset('AbsTol',1e-3, 'RelTol',5e-4);
        %
        tspanFull = linspace(tspan(1),tspan(2),1000);
      %  fprintf('Simulating time for simple step input ');

       % tic;
        [t, x] = ode15s(sys,tspanFull,xi,vopt);
      %  toc

        result(i).WHat = WHat;
        result(i).t = t;
        result(i).x = x;
        result(i).yOut = mechOpt.C*result(i).x';

        figure(3);
        plot(t,WHat*basisFunc(t,basisOpt),'Color',cols(i,:));hold on;
        plot(t,trainingFunc(t'),'--','Color',cols(i,:),'LineWidth',2);

        figure(5);
        %plot(t,mechOpt.C*x'); hold on;
        %trajectoryTrace_snapshot_noRobot(fkin(x(:,1),x(:,2),robotData()),t,robotData(),3,0,cols(i,:));hold on; grid on;

        plot(t,result(i).yOut,'Color',cols(i,:), 'LineWidth', 2.0); hold on; grid on;
        title('DMP Control'); xlabel('Time t(sec)'): ylabel('Position (m)');
        
        %% Checking dimensionality using empirical gramians
        %fprintf('Dim checking time : \n');
        basisOpt.order = basisOpt.N+1;
        %tic;  %                                        evaluateDimCost_NBT_HSV2Cost(WHat,tspan,mechSystem, mechOpt, basisOpt{j}, redThreshold, cm);              
        %[score, redOrder , redSys, hsvs, normHsvs ] = evaluateDimCost_BT_HSV2Cost(WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm);  
                                                  %    evaluateDimCost_BT_HSV2Cost(WHat,mechOpt,basisOpt{j},redThreshold,[],[],[],1);
        [score, redOrder , redSys, hsvs, normHsvs ] = evaluateDimCost_BT_HSV2Cost(WHat,mechOpt,basisOpt,redThreshold,[],[],[],1);
        
        inpSys = ss(mechOpt.A,mechOpt.B*WHat,mechOpt.C,mechOpt.D*WHat);
         hank = hsvd(inpSys)'
         normHank = (cumsum(hank)./sum(hank))
         mechOpt.B*WHat;
         
       % toc
       % WHat,mechOpt,basisOpt,threshold,redOrder,figNum,col,algoChoice
     %   score = 0;
     %   redOrder = 0;
     %   hsvs = zeros(4,1);
     
    %    hsvStore(i,:) = normHsvs;
        
        figure(6);
        subplot(2,2,i)
        bar(cumsum(hsvs)./sum(hsvs),'grouped','FaceColor',cols(i,:),'LineWidth',1.0);
        set(get(gca,'child'),'FaceColor',cols(i,:)); axis tight; grid on;
        title(sprintf('Trajectory : %d, k : %d',i,redOrder),'FontSize',14);
        
        %0.4,'FaceColor',cols(i,:)); axis tight;
       % set(gca,'XTickLabel',{'s1','s2','s3','s4'});%{'T1','T2','T3','T4','T5'});
%         set(gca,'yTick',fliplr([1,0.75,0.5,0.25,0]));
%         grid on;
%         set(gca,'FontSize',14);
         a = axis;axis([a(1:2) 0.5 a(4)]);line([a(1) a(2)]',[redThreshold redThreshold]','Color',[0 0 0],'LineWidth',2.0);
         
       % dxdt = planar2dofArm_red(t,y, ud, robot,Trans, invTrans, n,red_n)

%         figure(4);
%         plot(1:length(hsvs),cumsum(hsvs)./sum(hsvs),'-o','color',cols(i,:),'LineWidth',2); hold on; grid on; axis tight;
%         figure(10);
%         plot(t,result(i).yOut-ys,'Color',cols(i,:),'LineWidth',2.0); hold on; grid on;


      %  fprintf('\n redOrder : %d , score : %f\n', redOrder,score);
      %  disp(hsvs');
      %  disp(cumsum(hsvs)'./sum(hsvs));

       % hsvCostFuncStore(i,:) = score;
       % costFuncStore(i,:) = redOrder;

        result(i).hsv = hsvs;
        result(i).score = score;
        %result(i).normHsv = normHsvs;
       % result(i).redOrder = redOrder;
       % result(i).threshold = redThreshold;
       % result(i).redSys = redSys;
    end

%     
%     figure(5); bar(costFuncStore,'grouped','FaceColor',0.55*[1 1 1],'LineWidth',1.0); grid on;
%     figure(7); 
    
    
end    %figure(6); bar(hsvCostFuncStore,'grouped','FaceColor',0.65*[1 1 1],'LineWidth',1.0); grid on;
    figure(8);close(8);  
     figure(1);close(1);
%     figure(4);close(4); 
%     figure(5);close(5);
%     figure(6);close(6);
%     figure(7);close(7);
%     figure(9);close(9);  
%    %  figure(2);close(2);  
    figure(10);close(10);  