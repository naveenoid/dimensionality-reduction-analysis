%Script to optimise the dimensions on the rigidKinematicChainSystem

%{[0.7,0.2]',[0.6,0.4]',[0.6,0.2]',[0.4,0.4]',[0.4,0.6]'
%xDRange =  {[0.7,0.2]',[0.6,0.4]',[0.6,0.2]',[0.4,0.4]',[0.4,0.6]',[0.3,0.6]',[0.4,0.5]',[0.4,0.3]',[0.5,0.4]',[0.5,0.3]',[0.6,0.3]'};%,[0.3,0.6]',[0.3,0.6]'};%,...
           % [0.7,-0.2]',[0.6,-0.4]',[0.6,-0.2]',[0.4,-0.4]',[0.4,-0.6]'};
xDRange = {[0.6,0.2]',[0.7,0.1]'};%{[0.5,0.3]',[0.5,0.2]',[0.4,0.3]',[0.4,0.2]'};
% 
% xDRange ={[0.7,0.3]',...
%           [0.7,0.2]',...
%           [0.6,0.4]',...
%           [0.6,0.2]',...
%           [0.6,0.0]'
% };
% 
% xDRange ={[0.5,0.3]',...
%           [0.5,0.1]',...
%           [0.4,0.6]',...
%           [0.4,0.2]',...
%           [0.2,0.4]'
% };

% xDRange = { 
%     [0.6852,    0.2772]',...
%     [0.6237,    0.2427]',...
%     [0.5719,    0.1948]',...
%     [0.5327,    0.1362]',...
%     [0.5083,    0.0700]',...
%     [0.5000,    0.0000]'
% };

%{[0.5,0.4]'}; % {[0.5,0.4]',[0.5,0.4]',[0.5,0.4]',[0.5,0.4]'};
%,[0.3,-0.6]', [0.3,-0.7]', [0.2,-0.6]'};
%{[0.6,-0.2]',[0.4,-0.4]', [0.6,-0.4]',[0.2,-0.6]',[0.4,-0.6]'};%,...
            %[-0.6,-0.2]',[-0.4,-0.4]',[-0.6,-0.4]',[-0.2,-0.6]',[-0.4,-0.6]'};
          
tspanRange = 1.5;%1.5:0.5:2.0;%2.5:0.5:5.5;%5.25;%1.0:0.5:3.5;

exitFlagStore = zeros(length(xDRange),length(tspanRange));
            

 cols = [winter(length(xDRange)); autumn(length(xDRange)) ; cool(length(xDRange)+2)];
j = 0;
for tspanMax = tspanRange
    j = j+1;
    for i = 1:length(xDRange)
        clearvars -except i xDRange tspanRange tspanMax exitFlagStore j cols
        
        
        % Robot setup
        mechOpt = robotData();
        xi = mechOpt.initialCond;
        tspan = [0;tspanMax];%[0:0.001:10];%train.tspan;
        mechSystem = @planar2dofArm;
        mechOpt.inputDim = 2;
        mechOpt.stateDim = 4;
        mechOpt.outputDim = 2;
        mechOpt.outputs = [1,2];

        %desiredPos = [0.3,-0.7]';%[0.5, -0.5]';
        %desiredPos = [0.3, -0.7]';
        desiredPos = xDRange{i};%[0.3, -0.4]';%[0.5,-0.5]';%[0.3, -0.7]';

        desiredTheta = iKin(desiredPos', 'down',mechOpt);
% 
%         %xDes = [0.5 -0.5]';
% 
%         tauSteady = computeSteadyStateTorque(iKin(desiredPos','down',mechOpt)', mechOpt);
%         fprintf('Tau Computed down : '); tauSteady'
%         fprintf('\n');
% 
%         %trainingFunc = @(tTrain)( tauSteady*ones(1,length(tTrain)));
%         trainingFunc = @(tTrain)( tauSteady*ones(1,length(tTrain)));
%         

        tTrain =(tspan(1):0.01:tspan(2))';
        
        trainingFunc = generateBenchmarkTrajectoriesCartesian(desiredPos, tspan,i, mechOpt,cols(i+3*(j-1),:));
        % trainingFunc = @(tTrain)(tauSteady*(1+ tanh((tTrain'-0.15*max(tTrain))*15)));
        cm = 1.0;%1.75;%1.75;
        redThreshold = 0.975;%0.995;%0.987;
        mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);

        %polyBasis setup;
        basisFunc = @polyBasisNormalised;
        basisLearning = @polyBasisWeightLearningNormalised;
        basisOpt.order = 5;%6;%6;%6;
        basisOpt.tmax = tspan(2);

        
        uD = zeros(2,length(tTrain));
        uD = trainingFunc(tTrain);    
        
        %[WHat, basisOpt{j}] = basisLearning{j}(tTrain,uD',[],[],basisFunc{j},mechOpt,basisOpt{j});
        [WHat, basisOpt]= basisLearning(tTrain,uD',[],[],basisFunc,mechOpt,basisOpt);
        
        sprintf('Initial weights after sigmoid training : \n');
        disp(WHat);
        
        figure(8); 
        subplot(2,1,2);
        hold on;
        %func = basisOpt{j};
        plot(tTrain,WHat*basisFunc(tTrain, basisOpt),'color',cols(i+3*(j-1),:)); axis tight; grid on;
        xlabel('time (sec)');
        ylabel('PolyBasis u');
                
        drawnow;
        %% loading initial from a stored result
        %load('./Data/RigidKinematicChainBasisInput/finalVelocity_1finalAccelation_1_target_4-6','WOpt');
        %WHat = (reshape(WOpt,[basisOpt.order+1,mechOpt.inputDim]))';clear WOpt;

        mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 

        mechBasisOpt = mechOpt;
        mechBasisOpt.inputDim = basisOpt.order+1;
        drawnow

        fprintf('Starting optimisation\n');
        drawnow;

        fprintf('InitialWeights ');
        WHat
        %[ score, redOrder, redSys, hsv , normHsv] =  evaluateDimCost_NBT(WHat, tspan,mechSystem,mechOpt, basisOpt, redThreshold, cm);
        [ score, redOrder, redSys, hsv , normHsv] =  evaluateDimCost_NBT_HSV2Cost(WHat, tspan,mechSystem,mechOpt, basisOpt, redThreshold, cm);


        %[ result ] = testOptimalSolution( WHat , redThreshold, cm, tspan, mechSystem, mechOpt, basisFunc, basisOpt);
        drawnow;
        fprintf('Initial conditions of optimisation\n');
        score
        redOrder
        % 
        % result(train).hsv = hsv;
        % result(train).score = score;
        % result(train).normHsv = normHsv;
        % result(train).redOrder = redOrder;
        % result(train).threshold = redThreshold;
        % result(train).redSys = redSys;

        constraintFunc  = @(WHat)reachConstraint( WHat, desiredTheta, tspan,mechSystem, mechOpt,basisFunc,basisOpt);
        %costFunc  = @(WHat)evaluateDimCost_NBT( WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm,redOrder );
        costFunc  = @(WHat)evaluateDimCost_NBT_HSV2Cost( WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm,redOrder );
         Wref = reshape(WHat',numel(WHat),1);
        %size(Wref)

        WIni = Wref;%zeros(numel(W),1);%reshape(W',numel(W),1);

      %  [ result ] = testOptimalSolution( WIni , WIni, redThreshold, cm, tspan, mechSystem, mechOpt, basisFunc, basisOpt);

        fprintf('Initial Constraint : ');constraintFunc(WIni)
        %size(WIni)
        %WOpt= fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon)
       % tic;
        %figure(1);

        lb = -50.0 *  ones(size(WHat));%125
        ub = 50.0 * ones(size(WHat));
        options = optimset;
        options = optimset(options,'Display', 'iter');
        %options = optimset(options,'Display', 'final');
        %options = optimset(options,'Algorithm', 'active-set');
        options = optimset(options,'MaxFunEvals', 3000);
        options = optimset(options,'Algorithm', 'interior-point');
        options = optimset(options,'TolCon',1e-2);%5e-2);

        options = optimset(options,'PlotFcns', {  @optimplotx @optimplotfval @optimplotconstrviolation });
        %options = optimset(options,'PlotFcns', {  @optimplotfval @optimplotconstrviolation });
         [WOpt,Fval,ExitFlag]= fmincon(costFunc,WIni,[],[],[],[],lb,ub,constraintFunc, options);
         %WOpt= fmincon(costFunc,WIni,[],[],[],[],[],[],constraintFunc, options);
%        % toc
% 
      %  ExitFlag = 2; WOpt = WIni;
        exitFlagStore(i,j) = ExitFlag;
        [ result ] = testOptimalSolution( WOpt , WIni, redThreshold, cm, tspan, mechSystem, mechOpt, basisFunc, basisOpt,cols(i+3*(j-1),:));
        
        if(mod(tspan(2),1) == 0)
            save(sprintf('./Data/PassiveCompliantKinematicChainBasisInput/MDReaching_tSpan_%d_target_%d_%d_down_order5',floor(tspan(2)),10*desiredPos(1),10*desiredPos(2)),'result','WOpt' ,'WIni', 'redThreshold','cm', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
        else
            save(sprintf('./Data/PassiveCompliantKinematicChainBasisInput/MDReaching_tSpan_%d_5_target_%d_%d_down_order5',floor(tspan(2)),10*desiredPos(1),10*desiredPos(2)),'result','WOpt' ,'WIni', 'redThreshold','cm', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
        end
           % save(sprintf(
    end
end

save(sprintf('./Data/PassiveCompliantKinematicChainBasisInput/MDReachingSummary2'),'exitFlagStore','tspanRange', 'xDRange');

exitFlagSummary = zeros(size(exitFlagStore,1)+1,size(exitFlagStore,2)+2);
exitFlagSummary(2:end,1:2) = cell2mat(xDRange)';
exitFlagSummary(1,3:end) = tspanRange;%cell2mat(xDRange)';
exitFlagSummary(2:end,3:end) = exitFlagStore;
disp(exitFlagSummary);
