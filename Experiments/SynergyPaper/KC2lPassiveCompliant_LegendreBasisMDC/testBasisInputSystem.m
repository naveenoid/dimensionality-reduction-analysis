%Script to train a simple trajectory on the compliantKinematicChainSystem


% Step1 setup robot
% Step2 setup polybasis system
% Step3 Train simple noninverse trajectory
% Step4 Test behaviour
% Step5 Plot trajectory results

% Robot setup
mechOpt = robotData();
xi = mechOpt.initialCond;
tspan = [0;10];%[0:0.001:10];%train.tspan;
mechSystem = @planar2dofArm;
mechOpt.inputDim = 2;
mechOpt.stateDim = 8;
mechOpt.outputDim = 2;
mechOpt.outputs = [1,2];

cm = 1.75;
redThreshold = 0.987;
mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);

%polyBasis setup;
basisFunc = @polyBasis;
basisLearning = @polyBasisWeightLearning;
basisOpt.order = 12; 

xDes = [0.5;
    -0.4];
tauSteady = computeSteadyStateTorque(iKin(xDes,'down',mechOpt));


trainingFunc{1} = @(tTrain)( 1.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
%trainingFunc{2} = @(tTrain)( 0.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
%trainingFunc{3} = @(tTrain)( 1.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
%trainingFunc{2} = @(tTrain)( sin(pi*(tTrain./(0.5*max(tTrain)))));%@(tTrain)( cos(pi*(tTrain./(max(tTrain)))));
%trainingFunc{3} = @(tTrain)( sin(pi*(tTrain./(max(tTrain)))));

trajectoryName = {'idenSig','baseAngSig','nonIdentSig', 'intrinsic'};

result= struct();
tTrain =(tspan(1):0.1:tspan(2))';

xD = zeros(2,length(tTrain));

for train = 1:length(trainingFunc)
 %   tic;
    fprintf('Learning Training Trajectory %d\n',train);
    % trainingData 

    xD(2,:) = trainingFunc{train}(tTrain);%0.5*(1 + tanh(tTrain-max(tTrain)*0.5));
    xD(1,:) = trainingFunc{1}(tTrain);
    result(train).xD = xD;
    %( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
    WHat = basisLearning(tTrain,xD,[],[],basisFunc,mechOpt,basisOpt);
    mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
    
    mechBasisOpt = mechOpt;
    mechBasisOpt.inputDim = basisOpt.order+1;
    %uFunc = @(t)WHat*basisFunc(t,basisOpt);

    sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

    %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
    %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
    vopt = odeset('InitialStep',1e-10,'MaxStep',2e-2, 'AbsTol',1, 'RelTol',1);
    %
    
    %tic;
    [t, x] = ode15s(sys,tspan,xi,vopt);
    %toc
    
    disp(size(x));
    
    result(train).WHat = WHat;
    result(train).t = t;
    result(train).x = x;


    fprintf('Empirical Reduction of Trajectory %d\n',train);
    drawnow;

    %% Checking dimensionality using empirical gramians

   [ score, redOrder, redSys, hsv , normHsv] =  evaluateDimCost_NBT(WHat,mechSystem, mechOpt, basisOpt, redThreshold, cm);
   
    disp(score)
    disp(redOrder)

    result(train).hsv = hsv;
    result(train).score = score;
    result(train).normHsv = normHsv;
    result(train).redOrder = redOrder;
    result(train).threshold = redThreshold;
    result(train).redSys = redSys;
end

fprintf('\nTesting Pure Mechanical System dimensionality\n');

%% testing pure mech system 
tempOpt.order = mechOpt.inputDim-1;
[ score, redOrder, redSys, hsv, normHsv ] =  evaluateDimCost_NBT(ones(mechOpt.inputDim),mechSystem, mechOpt, tempOpt, redThreshold, cm);

disp(score)
disp(redOrder)

result(train+1).score = score;
result(train+1).hsv = hsv;
result(train+1).normHsv = hsv;
result(train+1).redOrder  = redOrder;
result(train+1).redSys = redSys;

cols = {'r','b','g','k'};

redOrderList = zeros(size(result,2),1);
redScoreList = zeros(size(result,2),1);

for train = 1:length(trainingFunc)+1
   if(train<=length(trainingFunc))
        figure(1);
            subplot(2,1,1);
            plot(result(train).t,result(train).x(:,1:2),cols{train}); hold on;
            ylabel('Position');
            xlabel('time t(secs)');
            legend('q_1','q_2');
    % 
            subplot(2,1,2);
            %title('inputs computed by polybasis');
            plot(result(train).t,result(train).WHat*basisFunc(result(train).t,basisOpt),cols{train});hold on;%, tTrain,result(train).xD,'k'); hold on;

        figure(2);
            trajectoryTrace_snapshot(fkin(result(train).x(:,1),result(train).x(:,2),robotData()),result(train).t,robotData(),2,0);hold on;
   end
    figure(3);
        plot(1:mechOpt.stateDim,cumsum(result(train).hsv)./sum(result(train).hsv),strcat(cols{train},'-o')); hold on;
        plot(cumsum(ones(8,1)),redThreshold*ones(8,1));
        axis tight;
        title('Hankel Singular Values');
        xlabel('State');
        ylabel('Normalised HSV');
        
        redOrderList(train) = result(train).redOrder;
        redScoreList(train) = result(train).score;
end

figure(1); legend(trajectoryName{1:end});
figure(2); legend(trajectoryName{1:end});
figure(3); legend(trajectoryName{1:end});

figure(4);
bar(1:length(trainingFunc)+1, redOrderList);
%legend(trajectoryName{1:end});
xlabel('Trajectories');
ylabel('Reduced Dimensionality');

disp(redScoreList);

set(gca,'Xtick',1:length(trainingFunc)+1,'XTickLabel',trajectoryName)
