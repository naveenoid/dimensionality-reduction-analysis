% Robot setup
mechOpt = robotData();
xi = mechOpt.initialCond;
        mechSystem = @planar2dofArm;
        mechOpt.inputDim = 2;
        mechOpt.stateDim = 4;
        mechOpt.outputDim = 2;
        mechOpt.outputs = [1,2];

        %desiredPos = [0.3,-0.7]';%[0.5, -0.5]';
        %desiredPos = [0.3, -0.7]';
        desiredPos = [0.3, -0.4]';%[0.5,-0.5]';%[0.3, -0.7]';

        desiredTheta = iKin(desiredPos', 'down',mechOpt);
        
        tspan = [0,2];
        
        
        
        tTrain =(tspan(1):0.01:tspan(2))';
        
        trainingFunc = generateBenchmarkTrajectoriesCartesian(desiredPos, tspan,1, mechOpt,'r');
        % trainingFunc = @(tTrain)(tauSteady*(1+ tanh((tTrain'-0.15*max(tTrain))*15)));
        cm = 1.0;%1.75;%1.75;
        redThreshold = 0.975;%0.995;%0.987;
        mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);

        %polyBasis setup;
        basisFunc = @legendreBasis;
        basisLearning = @legendreBasisWeightLearning;
        basisOpt.order = 5;%6;%6;%6;
        basisOpt.tmax = tspan(2);

        
        uD = zeros(2,length(tTrain));
        uD = trainingFunc(tTrain);    
        
        %[WHat, basisOpt{j}] = basisLearning{j}(tTrain,uD',[],[],basisFunc{j},mechOpt,basisOpt{j});
        [WHat, basisOpt]= basisLearning(tTrain,uD',[],[],basisFunc,mechOpt,basisOpt);
        

legendrefit(uD(:,1)', basisOpt.order);
%figure;
legendrefit(uD(:,2)',basisOpt.order);

% %W1 = 
% [W, Y2, coeff, D] = legendrefit(ydes1(t), legendreOpt.order);
% 
% 
% [Psi, p] = legendreBasis(t,legendreOpt);

figure;
plot(tTrain,uD,'r','LineWidth',2.0); hold on;
%figure;
plot(tTrain,WHat*legendreBasis(tTrain,basisOpt),'--b','LineWidth',2.0);