%% Script to compare polynomial and fourier basis systems on a 2Dof Kinematic Chain
%Script to train a simple trajectory on the tethered Mass system
%tspanRange = 1:0.5:8;
tspanRange = 2.5;%4.75;4.75;%6.0;%4.75;%;5.5;%:6.0;%5;
figure(1);figure(2);
figure(3);
figure(4);figure(5);figure(6);figure(7); figure(11);
for tspanMax = tspanRange
    
    clearvars -except tspanMax tspanRange

    % Robot setup
    
    mechOpt = robotData();
    xi = mechOpt.initialCond;
   % tspan = [0;2];%[0:0.001:10];%train.tspan;
    mechSystem = @planar2dofArm;
    mechOpt.inputDim = 2;
    mechOpt.stateDim = 4;
    mechOpt.outputDim = 2;
    mechOpt.outputs = [1,2];

    cm = 1.0;%0.945;%1.75;
   % redThreshold = 0.95;%0.985;
    mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);


    tspan = [0;tspanMax];%[0:0.001:10];%train.tspan;
    numTestTraj = 4;%4;%4;%5;%5;%4;%4;

    numBasis = 1;%2;%2;%2;%2;
    costFuncHSV2Store= zeros(numTestTraj,numBasis);
    costFuncRedOrderStore = zeros(numTestTraj,numBasis);

    %cm = 1.75;
    redThreshold = 0.88;%0.925;%7;
    mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);

    %xDes = [0.3520   -0.7181]';%[0.2, -0.75]';%[0.5,-0.4]';
    %xDes = [0.4 -0.6]';

    trajectoryName = {'T1','T2','T3','T4','T5','$\delta_i$'};%{'straightLine', 'curvedPoly', 'S-Curve','OvershootCurve','intrinsic'};

    result= struct();
    tTrain =(tspan(1):0.1:tspan(2))'; %:0.05:

    uD = zeros(2,length(tTrain));

     %   tic;
    fprintf('Learning Training Trajectory\n');
        % trainingData 
   % xStart = [0;0]; 
    xD = [0.5,0.2]';%[0.5,0.2]',%[0.5,0.3]';%[0.5;-0.6];%[0.5;-0.5];
   xStart = mechOpt.initialCond(1:2);
  % xD = [0.4, -0.6]';
    numPoints  = 1;


    %cols = ['r','b','g','c','m','y','k']';
    %cols = hsv(numTestTraj+1);
    %cols = jet(numTestTraj);
    %cols = hot(numTestTraj+1);
    %cols = gray(numTestTraj+1);
    %cols = copper(numTestTraj+1);
    cols = lines(numTestTraj+1);
    %cols = summer(numTestTraj+1);


    fprintf('\nTesting Pure Mechanical System dimensionality\n');

    %% testing pure mech system 
    tempOpt.order = mechOpt.inputDim-1;
    [ score, redOrder, redSys, hsvs, normHsv ] =  evaluateDimCost_NBT_HSV2Cost(ones(mechOpt.inputDim),tspan,mechSystem, mechOpt, tempOpt, redThreshold, cm, []);
   % [score, redOrder , redSys, hsvs, normHsvs ] = evaluateDimCost_BT(ones(mechOpt.inputDim),mechOpt,tempOpt,redThreshold,[],[],[],1);

    score


  %  costFuncStore(numTestTraj+1,1) = score;
  %  costFuncStore(numTestTraj+1,2) = score;

    %cols = {'r','b','g','c','m','y','k'};

    resultMech.score = score;
    resultMech.hsv = hsvs;
    resultMech.normHsv = hsvs;
    resultMech.redOrder  = redOrder;
    resultMech.redSys = redSys;
    
   % basisOpt = cell();
   % inputDim = cell();

   it = 1;
    %polyBasis setup;
    basisFunc{it} = @legendreBasis;
    basisLearning{it} = @legendreBasisWeightLearning;
    basisOpt{it}.order = 5;%5;%5;% 5;%5;%12;
    basisOpt{it}.tmax = tspanMax;
    inputDim{it} = basisOpt{1}.order+1;
    it = it+1;

    %polyBasis setup;
    basisFunc{it} = @fourierBasis;
    basisLearning{it} = @fourierBasisWeightLearning;
    basisOpt{it}.fourierOrder = 3;

    trainTitles = {'Training','LegendreBasis','FourierBasis'};

    for j =1:numBasis
        for i = 1:numTestTraj
            %figure(1);figure(1); 
            silentFigure(1);
            hold on;%+ 5*(j-1));
            trainingFunc = generateBenchmarkTrajectoriesCartesian(xD, tspan,i, mechOpt,cols(i,:));

            if j == 1
                fprintf('\n LegendreBasis\n');
            else
                fprintf('\n FourierBasis\n');
            end
            %uD(1,:) = trainingFunc{1}(tTrain);    
            %uD(2,:) = trainingFunc{2}(tTrain);
            
            uD = trainingFunc(tTrain);
            
            
             %xD(2,:) = trainingFunc{train}(tTrain);%0.5*(1 + tanh(tTrain-max(tTrain)*0.5));
             %xD(1,:) = trainingFunc{1}(tTrain);
             result(i).uD = uD;
             %( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
             [WHat, basisOpt{j}] = basisLearning{j}(tTrain,uD',[],[],basisFunc{j},mechOpt,basisOpt{j});
             mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
            
             figure(8); 
                subplot(3,1,1+j);
                hold on;
                %func = basisOpt{j};
                plot(tTrain,WHat*basisFunc{j}(tTrain, basisOpt{j}),'color',cols(i,:)); axis tight;
                xlabel('time (sec)');
                ylabel(trainTitles{j+1});
            
            if(j>1)
                inputDim{j} = basisOpt{j}.order;
            end

            mechBasisOpt = mechOpt;
            mechBasisOpt.inputDim = inputDim{j};%basisOpt.order+1;
            %uFunc = @(t)WHat*basisFunc(t,basisOpt);


            sys = @(t,x)mechBasisSystem(t,x,basisFunc{j}(t,basisOpt{j}));

            %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
            %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
            vopt = odeset('AbsTol',1e-3, 'RelTol',5e-4);
            %
            tspanFull = linspace(tspan(1),tspan(2),5000);
            fprintf('Simulating time for simple step input ');

            tic;
            [t, x] = ode15s(sys,tspanFull,xi,vopt);
            toc

            result(i,j).WHat = WHat;
            result(i,j).t = t;
            result(i,j).x = x;

            fk = fkin(x(end,1),x(end,2),mechOpt);%fk(3:4)
            fprintf('Original Des Pos : '); uD'
            fprintf('\n');
            fprintf('Position Reached : ');  fk(3:4)%fKin(xEnd',mechOpt);  %mechOpt.C*x(end,:)' %fk(3:4)
            fprintf('\n');

            fprintf('Empirical Reduction of Trajectory\n');
            drawnow;

            %% Checking dimensionality using empirical gramians
            fprintf('Dim checking time : \n');
            tic;
         %   [score, redOrder , redSys, hsvs, normHsvs ] = evaluateDimCost_NBT(WHat,tspan,mechSystem, mechOpt, basisOpt{j}, redThreshold, cm,[]);
            [ score, redOrder, redSys, hsvs , normHsvs] =  evaluateDimCost_NBT_HSV2Cost(WHat,tspan,mechSystem, mechOpt, basisOpt{j}, redThreshold, cm);
%             score = 2;
%             redOrder = 1;
%             redSys = mechSystem;
%             hsvs = [0.5, 0.65, 0.75, 1.0]';
%             normHsvs = [0.5, 0.65, 0.75, 1.0]';
            toc

            costFuncHSV2Store(i,j) = score;
            costFuncRedOrderStore(i,j) = redOrder;

            result(i,j).hsv = hsvs;
            result(i,j).score = score;
            result(i,j).normHsv = normHsvs;
            result(i,j).redOrder = redOrder;
            result(i,j).threshold = redThreshold;
            result(i,j).redSys = redSys;

         %   redOrderList = [result.redOrder, resultMech.redOrder];
         %   redScoreList = [result.score, resultMech.score];
        end
    end


    plotCartesianTrajectories( result, resultMech, trajectoryName, xD,redThreshold, numBasis, numTestTraj, cols,mechOpt,costFuncHSV2Store,costFuncRedOrderStore,basisFunc, basisLearning, basisOpt, inputDim );
     save(sprintf('./Data/SynergyPaper/KC2lPassiveCompliant_BasisComparisonTSDA/TSDAComparison_tspan_%d',round(10*tspan(2))),'result', 'resultMech', 'trajectoryName', 'xD','redThreshold', 'numBasis', 'numTestTraj', 'cols','mechOpt','costFuncHSV2Store','costFuncRedOrderStore','basisFunc', 'basisLearning', 'basisOpt', 'inputDim');
end