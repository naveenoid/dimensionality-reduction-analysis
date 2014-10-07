%% Script to compare polynomial and fourier basis systems on a 2Dof  Kinematic Chain
%Script to train a simple trajectory on the tethered Mass system
%tspanRange = 1:0.5:8;
tspanRange = 5.0;%;5.5;%:6.0;%5;
figure(1);
%figure(2);
%figure(3);
%figure(4);
%figure(5);
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

    cm = 1.0;
    mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);

    tspan = [0;tspanMax];%[0:0.001:10];%train.tspan;
    numTestTraj = 4;%4;%5;%5;%4;%4;

   % numBasis = 1;%2;
    %costFuncHSV2Store= zeros(numTestTraj,1);
    %costFuncRedOrderStore = zeros(numTestTraj,1);

    redThreshold = 0.925;%7;
    mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);
    trajectoryName = {'T1','T2','T3','T4','T5','$\delta_i$'};%{'straightLine', 'curvedPoly', 'S-Curve','OvershootCurve','intrinsic'};

    result= struct();
    tTrain =(tspan(1):0.1:tspan(2))'; %:0.05:
    uD = zeros(2,length(tTrain));

    fprintf('Learning Training Trajectory\n');
    xD = [0.6,0.3]';
    xStart = mechOpt.initialCond(1:2);
    numPoints  = 1;
    cols = lines(numTestTraj+1);


    fprintf('\nTesting Pure Mechanical System dimensionality\n');

    %% testing pure mech system 
    tempOpt.order = mechOpt.inputDim-1;
    [ score, redOrder, redSys, hsvs, normHsv ] =  evaluateDimCost_NBT_HSV2Cost(ones(mechOpt.inputDim),tspan,mechSystem, mechOpt, tempOpt, redThreshold, cm, []);

    score

    resultMech.score = score;
    resultMech.hsv = hsvs;
    resultMech.normHsv = hsvs;
    resultMech.redOrder  = redOrder;
    resultMech.redSys = redSys;
    
   % basisOpt = cell();
   % inputDim = cell();

%    it = 1;
%     %polyBasis setup;
%     basisFunc{it} = @polyBasisNormalised;
%     basisLearning{it} = @polyBasisWeightLearningNormalised;
%     basisOpt{it}.order = 6;%5;% 5;%5;%12;
%     basisOpt{it}.tmax = tspanMax;
%     inputDim{it} = basisOpt{1}.order+1;
%     it = it+1;
% 
%     %polyBasis setup;
%     basisFunc{it} = @fourierBasis;
%     basisLearning{it} = @fourierBasisWeightLearning;
%     basisOpt{it}.fourierOrder = 5;%5;%5;%5;%12;  %a number between 3 and 8
%     %basisOpt{2}.order = 7;%12; 


    for j = 1%1:numBasis
        for i = 4%1:numTestTraj
            %figure(1);figure(1); 
            silentFigure(1);
            hold on;%+ 5*(j-1));
            trainingFunc = generateBenchmarkTrajectoriesCartesian(xD, tspan,i, mechOpt,cols(i,:));

            uD = trainingFunc(tTrain);

            result(i).uD = uD;
% 
%             [WHat, basisOpt{j}] = basisLearning{j}(tTrain,uD',[],[],basisFunc{j},mechOpt,basisOpt{j});
%             mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
%             inputDim{j} = basisOpt{j}.order;
% 
%             mechBasisOpt = mechOpt;
%             mechBasisOpt.inputDim = inputDim{j};%basisOpt.order+1;
% 
%             sys = @(t,x)mechBasisSystem(t,x,basisFunc{j}(t,basisOpt{j}));
            sys = @(t,x)mechDirectSystem(t,x,trainingFunc(t)');
            vopt = odeset('AbsTol',1e-2, 'RelTol',5e-3);

            tspanFull = linspace(tspan(1),tspan(2),2500);
            fprintf('Simulating time for simple step input ');

            tic;
            [t, x] = ode15s(sys,tspanFull,xi,vopt);
            toc

%             result(i,j).WHat = WHat;
%             result(i,j).t = t;
%             result(i,j).x = x;

            fk = fkin(x(1:end,1),x(1:end,2),mechOpt);
            fprintf('Original Des Pos : '); uD'
            fprintf('\n');
            fprintf('Position Reached : ');  fk(end,3:4)
            fprintf('\n');

            fprintf('Empirical Reduction of Trajectory\n');
            drawnow;
                        
            figure(2);
            %subplot(2,1,1); 
            plot(t,x(:,1), 'r','LineWidth',2); hold on;
            plot(t,x(:,2), 'b','LineWidth',2);
            axis tight;axis equal;
            grid on;
            %subplot(2,1,2);
            
            figure(1);
            hold on;
            plot(fk(:,3),fk(:,4),'LineWidth',2,'color','r');
            xlabel('Time t(sec)');
            ylabel('End Position (m)');
            axis tight;
            grid on;
% 
%             %% Checking dimensionality using empirical gramians
%             fprintf('Dim checking time : \n');
%             tic;
%             [ score, redOrder, redSys, hsvs , normHsvs] =  evaluateDimCost_NBT_HSV2Cost(WHat,tspan,mechSystem, mechOpt, basisOpt{j}, redThreshold, cm);
%             toc
% 
%             costFuncHSV2Store(i,j) = score;
%             costFuncRedOrderStore(i,j) = redOrder;
% 
%             result(i,j).hsv = hsvs;
%             result(i,j).score = score;
%             result(i,j).normHsv = normHsvs;
%             result(i,j).redOrder = redOrder;
%             result(i,j).threshold = redThreshold;
%             result(i,j).redSys = redSys;

        end
    end


%    plotCartesianTrajectories( result, resultMech, trajectoryName, xD,redThreshold, numBasis, numTestTraj, cols,mechOpt,costFuncHSV2Store,costFuncRedOrderStore,basisFunc, basisLearning, basisOpt, inputDim );
%    save(sprintf('./Data/PolyFourierBasisComparisonKinematicChain/basisComparisonCartesian_KC_tspan_%d',round(10*tspan(2))),'result', 'resultMech', 'trajectoryName', 'xD','redThreshold', 'numBasis', 'numTestTraj', 'cols','mechOpt','costFuncHSV2Store','costFuncRedOrderStore','basisFunc', 'basisLearning', 'basisOpt', 'inputDim');
end