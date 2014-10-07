%% Script to compare legendre and fourier basis systems on a tethered mass system
% Script to compare the dimensionality cost of passing viapoints in reaching
% a target for 2 kinds of temporal bases : Legendre polynomials and Fourier

% Figure list
%figure(1);
figure(2); %HSV2Cost Polar
%figure(3);
figure(4); % Trajectories
%figure(5);
%figure(6);
%figure(7);
figure(8); % training trajectories
%figure(10);
%figure(11);
%figure(12);

%% Simulation ranges
tspanMax= 3.0;%3.5;%3.5;%2.4%.0;3.5;%3.5;
viapointDistRange = 0.1;%0.3:0.2:0.7;
viapointOrientRange = 0:pi/4:2*pi;
numBasisTested = 2%;2;%2;

%% Result storing
hsv2CostStore = zeros(size(viapointDistRange),size(viapointOrientRange),numBasisTested);
redDimStore = zeros(size(viapointDistRange),size(viapointOrientRange), numBasisTested);
result= struct();

%% Simulation / Robot settings
mechOpt = init2DSpringyDampedMass(2);%robotData();
mechOpt.initialCond = zeros(size(mechOpt.A,1),1);

mechSystem = @sysLTIForced;
mechOpt.inputDim = size(mechOpt.B,2);
mechOpt.stateDim = size(mechOpt.A,1);
mechOpt.outputDim = size(mechOpt.C,1);
mechOpt.outputs = [1,2];
tspan = [0;tspanMax];%[0:0.001:10];%train.tspan;

%% Basis Function definitions
basisFunc{1} = @legendreBasis;
basisLearning{1} = @legendreBasisWeightLearning;
basisOpt{1}.order = 4;%12; 
basisOpt{1}.tmax = tspanMax;
inputDim{1} = basisOpt{1}.order+1;  

basisFunc{2} = @fourierBasis;
basisLearning{2} = @fourierBasisWeightLearning;
basisOpt{2}.fourierOrder = 4;%5;%4;%12;  %a number between 3 and 8


%% Commence iteration of simulation over test ranges
for viapointDistCtr = 1:length(viapointDistRange) 
    for viapointOrientCtr = 1:length(viapointOrientRange)
        clearvars -except tspanMax viapointDistCtr viapointDistRange viapointOrientCtr viapointOrientRange numBasisTested hsv2CostStore redDimStore mechOpt mechSystem tspan result basisFunc...
            basisLearning basisOpt inputDim 

            % Robot setup
            xi = mechOpt.initialCond;

            redThreshold = 0.975;
            mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);
            
            tTrain =(tspan(1):0.05:tspan(2))';

            uD = zeros(2,length(tTrain));
            fprintf('Learning Training Trajectory\n');
            xStart = [0;0]; 
            xD = [0.5;0.5];
            
            xMid = xD-viapointDistCtr*[sin(viapointOrientCtr);cos(viapointOrientCtr)];%[0.5;0.5];
            numPoints  = 1;

            cols = lines(numBasisTested+1);
            
            for basisCtr = 1:numBasisTested
                %for i = 1:numTestTraj
                    i=1;
                    
                    %% Generating training trajectory
                    figure(1); hold on;
                    trainingFunc = generateBenchmarkTrajectoriesCartesian_viaPoint(xStart,xMid,xD, tspan,mechOpt,cols(1,:));

                    uD(1,:) = trainingFunc{1}(tTrain);    
                    uD(2,:) = trainingFunc{2}(tTrain);

                    %% training basis with the trajectory
                    [WHat, basisOpt{basisCtr}] = basisLearning{basisCtr}(tTrain,uD,[],[],basisFunc{basisCtr},mechOpt,basisOpt{basisCtr});
                    mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
                    if(basisCtr>1)
                        inputDim{basisCtr} = basisOpt{basisCtr}.order;
                    end

                   % figure(8); 
                    set(0,'CurrentFigure',8);
                    subplot(3,1,1+basisCtr);
                    hold on;
                     
                    plot(tTrain,WHat*basisFunc{basisCtr}(tTrain, basisOpt{basisCtr}),'color',cols(1,:)); axis tight;
                    xlabel('time (sec)');
                    %ylabel(trainTitles{basisCtr+1});

                    mechBasisOpt = mechOpt;
                    mechBasisOpt.inputDim = inputDim{basisCtr};

                    disp(mechBasisOpt.B);

                    sys = @(t,x)mechBasisSystem(t,x,basisFunc{basisCtr}(t,basisOpt{basisCtr}));

                    %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
                    %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
                    %vopt = odeset('AbsTol',1e-1, 'RelTol',5e-2);
                    vopt = odeset('AbsTol',1e-3, 'RelTol',5e-4);
                    
                    %% Simulating the system with the trained weights
                    
                    tspanFull = linspace(tspan(1),tspan(2),2000);
                    fprintf('Simulating time for simple step input ');

                    tic;
                    [t, x] = ode15s(sys,tspanFull,xi,vopt);
                    toc

                    result(viapointDistCtr,viapointOrientCtr,basisCtr).WHat = WHat;
                    result(viapointDistCtr,viapointOrientCtr,basisCtr).t = t;
                    result(viapointDistCtr,viapointOrientCtr,basisCtr).x = x;


                    fprintf('Empirical Reduction of Trajectory\n');
                    drawnow;

                    %% Checking dimensionality using gramians
                    fprintf('Dim checking time : \n');
                    tic;
                    [score, redOrder , redSys, hsvs, normHsvs ] = evaluateDimCost_BT_HSV2Cost(WHat,mechOpt,basisOpt{basisCtr},redThreshold,[],[],[],1);
                    toc

                    fprintf('normhsvs:');
                    disp(normHsvs');

                    hsv2CostStore(viapointDistCtr,viapointOrientCtr,basisCtr) = score;
                    redDimStore(viapointDistCtr,viapointOrientCtr,basisCtr) = redOrder;

                    result(viapointDistCtr,viapointOrientCtr,basisCtr).hsv = hsvs;
                    result(viapointDistCtr,viapointOrientCtr,basisCtr).score = score;
                    result(viapointDistCtr,viapointOrientCtr,basisCtr).redOrder = redOrder;
                    
            end
            save(sprintf('./Data/SynergyPaper/TetheredMass2D_BasisComparisonTSDA/BasisComparison_MultiViapointRange_%d_Orientation_%d_tspan_%d',round(10*viapointDistCtr),round(10*viapointOrientCtr),round(10*tspan(2))),'result', 'xD', 'viapointDistRange', 'viapointOrientRange','redThreshold',  'cols','mechOpt', 'hsv2CostStore', 'redDimStore','basisFunc', 'basisLearning', 'basisOpt', 'inputDim','result',  'xD','redThreshold');
     end
end

plotCartesianTrajectories_multiViapoint( result, viapointDistRange, viapointOrientRange, cols, hsv2CostStore, redDimStore, mechOpt);
%numBasisTested, viaPointDistRange, viaPointOrientRange cols,mechOpt,costFuncStore, hsvCostFuncStore,basisFunc, basisLearning, basisOpt, inputDim );
%figure(4);