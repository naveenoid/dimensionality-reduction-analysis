%% Script to test a sigmoidal trajectory to reach a position in space

%Script to train a simple trajectory on the compliantKinematicChainSystem


% Step1 setup robot
% Step2 setup polybasis system
% Step3 Train simple noninverse trajectory
% Step4 Test behaviour
% Step5 Plot trajectory results

% Robot setup
mechOpt = robotData();
xi = mechOpt.initialCond;
tspan = [0;7.5];%[0:0.001:10];%train.tspan;
mechSystem = @planar2dofArm;
mechOpt.inputDim = 2;
mechOpt.stateDim = 4;
mechOpt.outputDim = 2;
mechOpt.outputs = [1,2];

cm = 1.75;
redThreshold = 0.995;
mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);

%polyBasis setup;
basisFunc = @polyBasisNormalised;
basisLearning = @polyBasisWeightLearningNormalised;
basisOpt.order = 6;%12; 
basisOpt.tmax = tspan(2);

%xDes = [0.3520   -0.7181]';%[0.2, -0.75]';%[0.5,-0.4]';
%xDes = [0.4 -0.6]';

%xDes = [0.5, -0.3]';
xDes = [0.3,-0.7]';
%xDes = [0.5 -0.5]';
%fprintf('xDes\n');
%disp(xDes);

tauSteady = computeSteadyStateTorque(iKin(xDes','up',mechOpt)', mechOpt);
fprintf('Tau Computed up : '); tauSteady'
fprintf('\n');

tauSteady = computeSteadyStateTorque(iKin(xDes','down',mechOpt)', mechOpt);
fprintf('Tau Computed down : '); tauSteady'
fprintf('\n');

% tauSteady = computeSteadyStateTorque(iKin(xDes','down',mechOpt)', mechOpt);
% fprintf('Tau Computed down : '); tauSteady'
% fprintf('\n');

%tauSteady = [1;1];%computeSteadyStateTorque(iKin(xDes','down',mechOpt)', mechOpt);
%fprintf('Tau Actual : '); tauSteady'
%fprintf('\n');

%trainingFunc = @(tTrain)( 1.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
trainingFunc = @(tTrain)( tauSteady*ones(1,length(tTrain)));
%trainingFunc = @(tTrain)( tauSteady*(1 + tanh(tTrain-max(tTrain)*0.5))')
%trainingFunc{2} = @(tTrain)( 0.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
%trainingFunc{3} = @(tTrain)( 1.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
%trainingFunc{2} = @(tTrain)( sin(pi*(tTrain./(0.5*max(tTrain)))));%@(tTrain)( cos(pi*(tTrain./(max(tTrain)))));
%trainingFunc{3} = @(tTrain)( sin(pi*(tTrain./(max(tTrain)))));

trajectoryName = {'reachStep', 'intrinsic'};

result= struct();
tTrain =(tspan(1):0.1:tspan(2))';

xD = zeros(2,length(tTrain));

 %   tic;
fprintf('Learning Training Trajectory\n');
    % trainingData 

xD = trainingFunc(tTrain);    
%xD(2,:) = trainingFunc{train}(tTrain);%0.5*(1 + tanh(tTrain-max(tTrain)*0.5));
%xD(1,:) = trainingFunc{1}(tTrain);
result.xD = xD;
%( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
WHat = basisLearning(tTrain,xD,[],[],basisFunc,mechOpt,basisOpt)
mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 

mechBasisOpt = mechOpt;
mechBasisOpt.inputDim = basisOpt.order+1;
%uFunc = @(t)WHat*basisFunc(t,basisOpt);


sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

%vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
%vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
vopt = odeset('AbsTol',1e-1, 'RelTol',5e-2);
%
tspanFull = linspace(tspan(1),tspan(2),500);
fprintf('Simulating time for simple step input ');

tic;
[t, x] = ode15s(sys,tspanFull,xi,vopt);
toc

result.WHat = WHat;
result.t = t;
result.x = x;

fk = fkin(x(end,1),x(end,2),mechOpt);
fprintf('Original Des Pos : '); xDes'
fprintf('\n');
fprintf('Position Reached : ');fk(3:4)
fprintf('\n');

fprintf('Empirical Reduction of Trajectory\n');
drawnow;

%% Checking dimensionality using empirical gramians
fprintf('Dim checking time : \n');
tic;
[ score, redOrder, redSys, hsv , normHsv] =  evaluateDimCost_NBT(WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm);
toc

score

result.hsv = hsv;
result.score = score;
result.normHsv = normHsv;
result.redOrder = redOrder;
result.threshold = redThreshold;
result.redSys = redSys;


fprintf('\nTesting Pure Mechanical System dimensionality\n');

%% testing pure mech system 
tempOpt.order = mechOpt.inputDim-1;
[ score, redOrder, redSys, hsv, normHsv ] =  evaluateDimCost_NBT_HSV2Cost(ones(mechOpt.inputDim),tspan,mechSystem, mechOpt, tempOpt, redThreshold, cm);

score
resultMech.score = score;
resultMech.hsv = hsv;
resultMech.normHsv = hsv;
resultMech.redOrder  = redOrder;
resultMech.redSys = redSys;

cols = {'r','b','g','k'};

redOrderList = zeros(size(result,2),1);
redScoreList = zeros(size(result,2),1);

figure(1);
subplot(2,1,1);
plot(result.t,result.x(:,1:2),cols{1}); hold on;
ylabel('Position');
xlabel('time t(secs)');
legend('q_1','q_2');
% 
subplot(2,1,2);
%title('inputs computed by polybasis');
plot(result.t,result.WHat*basisFunc(result.t,basisOpt),cols{1});hold on;%, tTrain,result(train).xD,'k'); hold on;

figure(2);
trajectoryTrace_snapshot(fkin(result.x(:,1),result.x(:,2),robotData()),result.t,robotData(),2,0);hold on;
figure(3);
plot(1:mechOpt.stateDim,cumsum(result.hsv)./sum(result.hsv),strcat(cols{1},'-o'),1:mechOpt.stateDim,cumsum(resultMech.hsv)./sum(resultMech.hsv),strcat(cols{2},'-o')); hold on;
plot(cumsum(ones(4,1)),redThreshold*ones(4,1));
axis tight;
title('Hankel Singular Values');
xlabel('State');
ylabel('Normalised HSV');

redOrderList = [result.redOrder, resultMech.redOrder];
redScoreList = [result.score, resultMech.score];

figure(1); legend(trajectoryName{1:end});
figure(2); legend(trajectoryName{1:end});
figure(3); legend(trajectoryName{1:end});

% fprintf('Testing Constraint Violation \n');
% tic;
% [ c,ceq ] = reachConstraint( result.WHat, xDes, tspan,mechSystem, mechOpt,basisFunc,basisOpt)
% toc

cartPos=fkin(result.x(:,1),result.x(:,2),robotData());
fprintf('End Pos : ');
cartPos(end,3:4);

cartVel=diff(cartPos);

fprintf('End Vel : ');
cartVel(end,3:4)
%disp(c);
% 
% figure(4);
% bar(1:length(trainingFunc)+1, redOrderList);
% %legend(trajectoryName{1:end});
% xlabel('Trajectories');
% ylabel('Reduced Dimensionality');

%disp(redScoreList);
%disp(redOrderList);

%set(gca,'Xtick',1:length(trainingFunc)+1,'XTickLabel',trajectoryName)



clear all; 



%% Script to test a sigmoidal trajectory to reach a position in space

%Script to train a simple trajectory on the compliantKinematicChainSystem


% Step1 setup robot
% Step2 setup polybasis system
% Step3 Train simple noninverse trajectory
% Step4 Test behaviour
% Step5 Plot trajectory results

% Robot setup
mechOpt = robotData();
xi = mechOpt.initialCond;
tspan = [0;7.5];%[0:0.001:10];%train.tspan;
mechSystem = @planar2dofArm;
mechOpt.inputDim = 2;
mechOpt.stateDim = 4;
mechOpt.outputDim = 2;
mechOpt.outputs = [1,2];

cm = 1.75;
redThreshold = 0.995;
mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);

%polyBasis setup;
basisFunc = @polyBasis;
basisLearning = @polyBasisWeightLearning;
basisOpt.order = 6;%12; 
basisOpt.tmax = tspan(2);

%xDes = [0.3520   -0.7181]';%[0.2, -0.75]';%[0.5,-0.4]';
%xDes = [0.4 -0.6]';

%xDes = [0.5, -0.3]';
xDes = [0.3,-0.7]';
%xDes = [0.5 -0.5]';
%fprintf('xDes\n');
%disp(xDes);

tauSteady = computeSteadyStateTorque(iKin(xDes','up',mechOpt)', mechOpt);
fprintf('Tau Computed up : '); tauSteady'
fprintf('\n');

tauSteady = computeSteadyStateTorque(iKin(xDes','down',mechOpt)', mechOpt);
fprintf('Tau Computed down : '); tauSteady'
fprintf('\n');

% tauSteady = computeSteadyStateTorque(iKin(xDes','down',mechOpt)', mechOpt);
% fprintf('Tau Computed down : '); tauSteady'
% fprintf('\n');

%tauSteady = [1;1];%computeSteadyStateTorque(iKin(xDes','down',mechOpt)', mechOpt);
%fprintf('Tau Actual : '); tauSteady'
%fprintf('\n');

%trainingFunc = @(tTrain)( 1.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
trainingFunc = @(tTrain)( tauSteady*ones(1,length(tTrain)));
%trainingFunc = @(tTrain)( tauSteady*(1 + tanh(tTrain-max(tTrain)*0.5))')
%trainingFunc{2} = @(tTrain)( 0.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
%trainingFunc{3} = @(tTrain)( 1.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
%trainingFunc{2} = @(tTrain)( sin(pi*(tTrain./(0.5*max(tTrain)))));%@(tTrain)( cos(pi*(tTrain./(max(tTrain)))));
%trainingFunc{3} = @(tTrain)( sin(pi*(tTrain./(max(tTrain)))));

trajectoryName = {'reachStep', 'intrinsic'};

result= struct();
tTrain =(tspan(1):0.1:tspan(2))';

xD = zeros(2,length(tTrain));

 %   tic;
fprintf('Learning Training Trajectory\n');
    % trainingData 

xD = trainingFunc(tTrain);    
%xD(2,:) = trainingFunc{train}(tTrain);%0.5*(1 + tanh(tTrain-max(tTrain)*0.5));
%xD(1,:) = trainingFunc{1}(tTrain);
result.xD = xD;
%( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
WHat = basisLearning(tTrain,xD,[],[],basisFunc,mechOpt,basisOpt)
mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 

mechBasisOpt = mechOpt;
mechBasisOpt.inputDim = basisOpt.order+1;
%uFunc = @(t)WHat*basisFunc(t,basisOpt);


sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

%vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
%vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
vopt = odeset('AbsTol',1e-1, 'RelTol',5e-2);
%
tspanFull = linspace(tspan(1),tspan(2),500);
fprintf('Simulating time for simple step input ');

tic;
[t, x] = ode15s(sys,tspanFull,xi,vopt);
toc

result.WHat = WHat;
result.t = t;
result.x = x;

fk = fkin(x(end,1),x(end,2),mechOpt);
fprintf('Original Des Pos : '); xDes'
fprintf('\n');
fprintf('Position Reached : ');fk(3:4)
fprintf('\n');

fprintf('Empirical Reduction of Trajectory\n');
drawnow;

%% Checking dimensionality using empirical gramians
fprintf('Dim checking time : \n');
tic;
[ score, redOrder, redSys, hsv , normHsv] =  evaluateDimCost_NBT(WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm);
toc

score

result.hsv = hsv;
result.score = score;
result.normHsv = normHsv;
result.redOrder = redOrder;
result.threshold = redThreshold;
result.redSys = redSys;


fprintf('\nTesting Pure Mechanical System dimensionality\n');

%% testing pure mech system 
tempOpt.order = mechOpt.inputDim-1;
[ score, redOrder, redSys, hsv, normHsv ] =  evaluateDimCost_NBT_HSV2Cost(ones(mechOpt.inputDim),tspan,mechSystem, mechOpt, tempOpt, redThreshold, cm);

score
resultMech.score = score;
resultMech.hsv = hsv;
resultMech.normHsv = hsv;
resultMech.redOrder  = redOrder;
resultMech.redSys = redSys;

cols = {'r','b','g','k'};

redOrderList = zeros(size(result,2),1);
redScoreList = zeros(size(result,2),1);

figure(4);
subplot(2,1,1);
plot(result.t,result.x(:,1:2),cols{1}); hold on;
ylabel('Position');
xlabel('time t(secs)');
legend('q_1','q_2');
% 
subplot(2,1,2);
%title('inputs computed by polybasis');
plot(result.t,result.WHat*basisFunc(result.t,basisOpt),cols{1});hold on;%, tTrain,result(train).xD,'k'); hold on;

figure(5);
trajectoryTrace_snapshot(fkin(result.x(:,1),result.x(:,2),robotData()),result.t,robotData(),5,0);hold on;
figure(6);
plot(1:mechOpt.stateDim,cumsum(result.hsv)./sum(result.hsv),strcat(cols{1},'-o'),1:mechOpt.stateDim,cumsum(resultMech.hsv)./sum(resultMech.hsv),strcat(cols{2},'-o')); hold on;
plot(cumsum(ones(4,1)),redThreshold*ones(4,1));
axis tight;
title('Hankel Singular Values');
xlabel('State');
ylabel('Normalised HSV');

redOrderList = [result.redOrder, resultMech.redOrder];
redScoreList = [result.score, resultMech.score];

figure(4); legend(trajectoryName{1:end});
figure(5); legend(trajectoryName{1:end});
figure(6); legend(trajectoryName{1:end});

% fprintf('Testing Constraint Violation \n');
% tic;
% [ c,ceq ] = reachConstraint( result.WHat, xDes, tspan,mechSystem, mechOpt,basisFunc,basisOpt)
% toc

cartPos=fkin(result.x(:,1),result.x(:,2),robotData());
fprintf('End Pos : ');
cartPos(end,3:4);

cartVel=diff(cartPos);

fprintf('End Vel : ');
cartVel(end,3:4)
%disp(c);
% 
% figure(4);
% bar(1:length(trainingFunc)+1, redOrderList);
% %legend(trajectoryName{1:end});
% xlabel('Trajectories');
% ylabel('Reduced Dimensionality');

%disp(redScoreList);
%disp(redOrderList);

%set(gca,'Xtick',1:length(trainingFunc)+1,'XTickLabel',trajectoryName)


