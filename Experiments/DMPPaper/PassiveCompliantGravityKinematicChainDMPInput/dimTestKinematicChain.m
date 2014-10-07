%% Script to test How the DMP basis system works with the kinematic chain mech system.
figure(1);
    mechOpt = robotData();
    xi = mechOpt.initialCond;
 tspan = [0;1];%[0:0.001:10];%train.tspan;
    mechSystem = @planar2dofArm;
    mechOpt.inputDim = 2;
    mechOpt.stateDim = 4;
    mechOpt.outputDim = 2;
    mechOpt.outputs = [1,2];

    cm = 1.0;%0.945;%1.75;
   redThreshold = 0.985;%0.994;%0.985;
    mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);
    
   numTestTraj = 4;%4;%5;%4;%4;
    cols = colormap(hsv(numTestTraj+1));
    
   trajectoryName = {'T1','T2','T3','T4','\Delta_i'};%{'straightLine', 'curvedPoly', 'S-Curve','OvershootCurve','intrinsic'};

      fprintf('Learning Training Trajectory\n');
        % trainingData 
   % xStart = [0;0]; 
    xD = [0.5,-0.5]';%[0.5;-0.6];%[0.5;-0.5];
    
    desiredPos = xD;
   xStart = mechOpt.initialCond(1:2);
   
    result= struct();
    tTrain =(tspan(1):0.05:tspan(2))';

    uD = zeros(2,length(tTrain));

     %   tic;
    fprintf('Learning Training Trajectory\n');
        % trainingData 
    %xStart = [0]; 
   % xD = [0.5];
    %numPoints  = 1;

    basisOpt.N= 25;%50;%12; 
    basisOpt.rescale = 10;%10000;
    %basisOpt.order = basisOpt.N;
    load(sprintf('./Data/DMPPaper/DMPBasis/psiStar_N%d',basisOpt.N),'PsiStar','tOut');
    
    PsiStar(1:basisOpt.N,:) = PsiStar(1:basisOpt.N,:).*basisOpt.rescale;
    
    %polyBasis setup;
    basisFunc= @dmpDataSys;
    basisLearning = @dmpBasisWeightLearning;
    
    basisOpt.alphaZ = 25;
    basisOpt.tDes = tOut;
    
 %   basisOpt.N = 10;
    
    inputDim = basisOpt.N+2;
    
   % cols = winter(length(trajectoryName)); 
   
   hsvStore = zeros(4,4);
    
    for i = 1:numTestTraj
    
        silentFigure(1);
        hold on;%+ 5*(j-1));
        %trainingFunc = generateBenchmarkTrajectories(xD, tspan,i, mechOpt,cols(i,:));

        trainingFunc = generateBenchmarkTrajectoriesCartesian_KC(desiredPos, tspan,i, mechOpt,cols(i,:));
      % trainingFunc = @(tTrain)(ones(5,1)*sin(10*tTrain'));%generateBenchmarkTrajectories(xStart,xD, tspan,i, mechOpt,cols(i,:));
      % trainingFunc = @(tTrain)(ones(5,1)*0.5*(1 + tanh(tTrain'-max(tTrain)*0.5)));%generateBenchmarkTrajectories(xStart,xD, tspan,i, mechOpt,cols(i,:));
      % trainingFunc = @(tTrain)(ones(5,1)*0.5*(1 + sinh(tTrain'-max(tTrain)*0.5)));%generateBenchmarkTrajectories(xStart,xD, tspan,i, mechOpt,cols(i,:));

       % uD(1,:) = trainingFunc(tTrain);    
       % uD(2,:) = trainingFunc(tTrain);    
       % uD(3,:) = trainingFunc(tTrain);    
       % uD(4,:) = trainingFunc(tTrain);    
       % uD(5,:) = trainingFunc(tTrain);    
       % uD(2,:) = trainingFunc{2}(tTrain);

     %   result.uD = uD;
     % ( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
     % [WHat, basisOpt] = dmpLearning(tTrain, mechOpt.inputDim, trainingFunc,'nosmooth', basisOpt) ;%tTrain,uD,[],[],basisFunc{j},mechOpt,basisOpt{j});
       [WHat, basisOpt] = dmpPInvLearn(tOut,PsiStar,trainingFunc,'nosmooth',basisOpt);
        %basisOpt.psiStar = PsiStar;
        basisOpt.tDes = linspace(min(tspan),max(tspan),length(tOut));%tOut;
        mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
        %inputDim = basisOpt.order;

        mechBasisOpt = mechOpt;
        mechBasisOpt.inputDim = basisOpt.N+2;%basisOpt.order+1;
        %uFunc = @(t)WHat*basisFunc(t,basisOpt);


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
        fprintf('Simulating time for simple step input ');

        tic;
        [t, x] = ode15s(sys,tspanFull,xi,vopt);
        toc

        result(i).WHat = WHat;
        result(i).t = t;
        result(i).x = x;

        figure(2);
        plot(t,WHat*basisFunc(t,basisOpt));hold on;
        plot(t,trainingFunc(t),'r');

        figure(3);
        %plot(t,mechOpt.C*x'); hold on;
        trajectoryTrace_snapshot_noRobot(fkin(x(:,1),x(:,2),robotData()),t,robotData(),3,0,cols(i,:));hold on; grid on;

        
        %% Checking dimensionality using gramians
        fprintf('Dim checking time : \n');
        basisOpt.order = basisOpt.N+1;
        tic;  %                                        evaluateDimCost_NBT_HSV2Cost(WHat,tspan,mechSystem, mechOpt, basisOpt{j}, redThreshold, cm);              
        [score, redOrder , redSys, hsvs, normHsvs ] = evaluateDimCost_NBT_HSV2Cost(WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm);  
        toc
     %   score = 0;
     %   redOrder = 0;
     %   hsvs = zeros(4,1);
     
        hsvStore(i,:) = normHsvs;
        
        figure(7);
        subplot(2,2,i)
        bar(normHsvs,'grouped','FaceColor',cols(i,:),'LineWidth',1.0);
        set(get(gca,'child'),'FaceColor',cols(i,:)); axis tight;
        %0.4,'FaceColor',cols(i,:)); axis tight;
        set(gca,'XTickLabel',{'s1','s2','s3','s4'});%{'T1','T2','T3','T4','T5'});
       % set(gca,'yTick',fliplr([1,0.75,0.5,0.25,0]));
        grid on;
        set(gca,'FontSize',14);
        a = axis;axis([a(1:2) 0.8 a(4)]);line([a(1) a(2)]',[redThreshold redThreshold]','Color',[0 0 0],'LineWidth',2.0);
        
       % dxdt = planar2dofArm_red(t,y, ud, robot,Trans, invTrans, n,red_n)

        figure(4);
        plot(1:length(hsvs),cumsum(hsvs)./sum(hsvs),'-o','color',cols(i,:),'LineWidth',2); hold on; grid on; axis tight;

        fprintf('\n redOrder : %d , score : %f\n', redOrder,score);

        hsvCostFuncStore(i,:) = score;
        costFuncStore(i,:) = redOrder;

        result(i).hsv = hsvs;
        result(i).score = score;
        %result(i).normHsv = normHsvs;
       % result(i).redOrder = redOrder;
       % result(i).threshold = redThreshold;
       % result(i).redSys = redSys;
    end

    
    figure(5); bar(costFuncStore,'grouped','FaceColor',0.55*[1 1 1],'LineWidth',1.0); grid on;
    figure(6); plot(tOut,PsiStar(1:basisOpt.N,:),'LineWidth',2.0);
    figure(7); plot(tOut,PsiStar(basisOpt.N+1:basisOpt.N+2,:),'LineWidth',2.0);
    
    
    
    %figure(6); bar(hsvCostFuncStore,'grouped','FaceColor',0.65*[1 1 1],'LineWidth',1.0); grid on;
    