%% Script to compare polynomial and fourier basis systems on a tethered mass system
%Script to train a simple trajectory on the tethered Mass system

% Robot setup
mechOpt = init2DSpringyDampedMass(2);%robotData();
mechOpt.initialCond = zeros(size(mechOpt.A,1),1);
xi = mechOpt.initialCond;
tspan = [0;2.5];%[0:0.001:10];%train.tspan;
mechSystem = @sysLTIForced;
mechOpt.inputDim = size(mechOpt.B,2);
mechOpt.stateDim = size(mechOpt.A,1);
mechOpt.outputDim = size(mechOpt.C,1);
mechOpt.outputs = [1,2];

numTestTraj = 4;

numBasis = 2;
costFuncStore= zeros(numTestTraj+1,numBasis);

%cm = 1.75;
redThreshold = 0.975;
mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);

%xDes = [0.3520   -0.7181]';%[0.2, -0.75]';%[0.5,-0.4]';
%xDes = [0.4 -0.6]';

trajectoryName = {'T1','T2','T3','T4','\Delta_i'};%{'straightLine', 'curvedPoly', 'S-Curve','OvershootCurve','intrinsic'};

result= struct();
tTrain =(tspan(1):0.1:tspan(2))';

uD = zeros(2,length(tTrain));

 %   tic;
fprintf('Learning Training Trajectory\n');
    % trainingData 
xStart = [0;0]; 
xD = [0.5;0.5];
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
%[ score, redOrder, redSys, hsv, normHsv ] =  evaluateDimCost_NBT(ones(mechOpt.inputDim),tspan,mechSystem, mechOpt, tempOpt, redThreshold, cm);
[score, redOrder , redSys, hsvs, normHsvs ] = evaluateDimCost_BT(ones(mechOpt.inputDim),mechOpt,tempOpt,redThreshold,[],[],[],1);

score


costFuncStore(numTestTraj+1,1) = score;
costFuncStore(numTestTraj+1,2) = score;

%cols = {'r','b','g','c','m','y','k'};

resultMech.score = score;
resultMech.hsv = hsvs;
resultMech.normHsv = hsvs;
resultMech.redOrder  = redOrder;
resultMech.redSys = redSys;


%polyBasis setup;
basisFunc{1} = @polyBasis;
basisLearning{1} = @polyBasisWeightLearning;
basisOpt{1}.order = 5;%12; 
inputDim{1} = basisOpt{1}.order+1;


%polyBasis setup;
basisFunc{2} = @fourierBasis;
basisLearning{2} = @fourierBasisWeightLearning;
basisOpt{2}.fourierOrder = 5;%12;  %a number between 3 and 8
%basisOpt{2}.order = 7;%12; 


for j = 1:numBasis
    for i = 1:numTestTraj
        figure(1); hold on;%+ 5*(j-1));
        trainingFunc = generateBenchmarkTrajectories(xStart,xD, tspan,i, mechOpt,cols(i,:));

        uD(1,:) = trainingFunc{1}(tTrain);    
        uD(2,:) = trainingFunc{2}(tTrain);

        %xD(2,:) = trainingFunc{train}(tTrain);%0.5*(1 + tanh(tTrain-max(tTrain)*0.5));
        %xD(1,:) = trainingFunc{1}(tTrain);
        result(i).uD = uD;
        %( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
        [WHat, basisOpt{j}] = basisLearning{j}(tTrain,uD,[],[],basisFunc{j},mechOpt,basisOpt{j});
        mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
        inputDim{j} = basisOpt{j}.order;
        
        mechBasisOpt = mechOpt;
        mechBasisOpt.inputDim = inputDim{j};%basisOpt.order+1;
        %uFunc = @(t)WHat*basisFunc(t,basisOpt);


        sys = @(t,x)mechBasisSystem(t,x,basisFunc{j}(t,basisOpt{j}));

        %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
        %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
        vopt = odeset('AbsTol',1e-1, 'RelTol',5e-2);
        %
        tspanFull = linspace(tspan(1),tspan(2),500);
        fprintf('Simulating time for simple step input ');

        tic;
        [t, x] = ode15s(sys,tspanFull,xi,vopt);
        toc

        result(i,j).WHat = WHat;
        result(i,j).t = t;
        result(i,j).x = x;

        %fk = fkin(x(end,1),x(end,2),mechOpt);
        fprintf('Original Des Pos : '); uD'
        fprintf('\n');
        fprintf('Position Reached : '); mechOpt.C*x(end,:)' %fk(3:4)
        fprintf('\n');

        fprintf('Empirical Reduction of Trajectory\n');
        drawnow;

        %% Checking dimensionality using empirical gramians
        fprintf('Dim checking time : \n');
        tic;
        [score, redOrder , redSys, hsvs, normHsvs ] = evaluateDimCost_BT(WHat,mechOpt,basisOpt{j},redThreshold,[],[],[],1);
        toc

        costFuncStore(i,j) = score;

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


plotCartesianTrajectories( result, resultMech, trajectoryName, xD,redThreshold, numBasis, numTestTraj, cols,mechOpt,costFuncStore,basisFunc, basisLearning, basisOpt, inputDim );
save(sprintf('./Data/PolyFourierBasisComparison/basisComparison_tspan_%d',round(10*tspan(2))),'result', 'resultMech', 'trajectoryName', 'xD','redThreshold', 'numBasis', 'numTestTraj', 'cols','mechOpt','costFuncStore','basisFunc', 'basisLearning', 'basisOpt', 'inputDim','result', 'resultMech', 'trajectoryName', 'xD','redThreshold', 'numBasis', 'numTestTraj', 'cols','mechOpt','costFuncStore','basisFunc', 'basisLearning', 'basisOpt', 'inputDim');