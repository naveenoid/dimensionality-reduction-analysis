%% Optimising the dimensionality of a Fourier Basis System to reach a point in space


basisOrderRange = 4:1:6;%5:1:6;%4:1:6;%:5;%4:1:7;%5:1:8;
tspanRange = 0.8:0.2:1.2;% 17.5:2.5:20;%10:2.5:15;%10:2.5:15;%5:0.5:7;%:5:40;%4;%3.0:0.5:4.5;%1:0.5:3;%8
endPointRange = 0.5%0.3[0.3,0.7];% 0.5;0.6:0.2:1.0;%0.1:0.1:0.5;
exitFlagMat = zeros(length(basisOrderRange),length(tspanRange));
i = 0;

for endPoint = endPointRange
   for basisOrder = basisOrderRange 
       i = i+1; j=0; 
        for tspanMax = tspanRange
           
           j = j+1;
            fprintf('\n-----------------------------------------------------------\n');
            fprintf('TRIAL : optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint));
            fprintf('\n-----------------------------------------------------------\n');
            
            clearvars -except tspanRange tspanMax basisOrderRange basisOrder endPointRange endPoint exitFlagMat i j
    % 
    % 
    % tspanRange = 1:0.5:8
    % 
    % for tspanMax = tspanRange
    %   %  tspanmax = 3;
    %   clearvars -except tspanMax tspanRange

        % Robot setup
            mechOpt = init2DSpringyDampedMass(2);%robotData();
            mechOpt.initialCond = zeros(size(mechOpt.A,1),1);
            xi = mechOpt.initialCond;
            tspan = [0;tspanMax];%[0:0.001:10];%train.tspan;
            mechSystem = @sysLTIForced;
            mechOpt.inputDim = size(mechOpt.B,2);
            mechOpt.stateDim = size(mechOpt.A,1);
            mechOpt.outputDim = size(mechOpt.C,1);
            mechOpt.outputs = [1,2];
            
            initialTrajectoryType = 2; % 1-step, 2-sigmoid

            %cm = 1.75;
            redThreshold = 0.975;
            mechDirectSystem = @(t,x,u)mechSystem(t,x,u,mechOpt);


            %FourierBasis setup;
            basisFunc = @fourierBasis;
            basisLearning = @fourierBasisWeightLearning;
            basisOpt.fourierOrder = basisOrder;%12;  %a number between 3 and 8

            %xDes = [0.3520   -0.7181]';%[0.2, -0.75]';%[0.5,-0.4]';
            %xDes = [0.4 -0.6]';

            %xDes = [0.5, -0.3]';
            xDes = [endPoint,endPoint]';
            %xDes = [0.5 -0.5]';
            %fprintf('xDes\n');
            %disp(xDes);
            
            result= struct();
            tTrain =(tspan(1):0.05:tspan(2))';
            xD = zeros(2,length(tTrain));
            cols = lines(4+1);

             %   tic;
            fprintf('Learning Training Trajectory\n');

            if(initialTrajectoryType ==1)
            
            	FSteady = computeSteadyStateForce(xDes,mechOpt);
                trainingFunc = @(tTrain)( FSteady*ones(1,length(tTrain)));
                trajectoryName = {'reachStep', 'intrinsic'};
                xD = trainingFunc(tTrain); 
            else
                
                trajectoryName = {'reachSigmoid', 'intrinsic'};                
                trainingFunc = generateBenchmarkTrajectoriesCartesian([0,0]',xDes, tspan,1, mechOpt,cols(1,:)); %learning smooth sigmoid
                uD(1,:) = trainingFunc{1}(tTrain');    
                uD(2,:) = trainingFunc{2}(tTrain');

                xD = uD;
                
            end

            result.xD = xD;
            %( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
            [WHat,basisOpt] = basisLearning(tTrain,xD,[],[],basisFunc,mechOpt,basisOpt);
            mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 

        %   fprintf('Tspan : '); disp(tspanMax);
        %   fprintf('Omega : '); disp(basisOpt.w);
        %   fprintf('Ratio : '); disp(tspanMax / basisOpt.w);
           
            mechBasisOpt = mechOpt;
            mechBasisOpt.inputDim = basisOpt.order;
         
            uFunc = @(t)WHat*basisFunc(t,basisOpt);
            uF = uFunc(tTrain);
            
%             figure(8);
%             hold on;
%            % subplot(2,1,1); hold on;
%             plot(tTrain, uF(1,:),'r');
%           %  subplot(2,1,2); hold on;
%             plot(tTrain, uF(2,:),'r');
            

         %   fprintf('Num of basis funcs : %d\n',basisOpt.order );


            sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

            %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
            %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
            vopt = odeset('AbsTol',1e-1, 'RelTol',5e-2);
            %
            tspanFull = linspace(tspan(1),tspan(2),500);
           % fprintf('Simulating time for simple step input ');


            fprintf('Starting optimisation\n');
            drawnow;

            %fprintf('InitialWeights ');
            %WHat
            %[ score, redOrder, redSys, hsv , normHsv] =  evaluateDimCost_NBT(WHat, tspan,mechSystem,mechOpt, basisOpt, redThreshold, cm);
            [score, redOrder , redSys, hsv, normHsv ] = evaluateDimCost_BT_HSV2Cost(WHat,mechOpt,basisOpt,redThreshold,[],[],[],1);

            %[ result ] = testOptimalSolution( WHat , redThreshold, cm, tspan, mechSystem, mechOpt, basisFunc, basisOpt);
            drawnow;
            %fprintf('Initial conditions of optimisation\n');
            %score
            %redOrder
            % 
            % result(train).hsv = hsv;
            % result(train).score = score;
            % result(train).normHsv = normHsv;
            % result(train).redOrder = redOrder;
            % result(train).threshold = redThreshold;
            % result(train).redSys = redSys;

            %constraintFunc  = @(WHat)reachConstraint( WHat, desiredTheta, tspan,mechSystem, mechOpt,basisFunc,basisOpt);
            constraintFunc  = @(WHat)reachConstraint( WHat, xDes,mechSystem,mechOpt,tspan, basisFunc,basisOpt );
            %costFunc  = @(WHat)evaluateDimCost_NBT( WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm,redOrder );
            costFunc = @(WHat)evaluateDimCost_BT_HSV2Cost(WHat,mechOpt,basisOpt,redThreshold,[],[],[],1);
             Wref = reshape(WHat',numel(WHat),1);
            %size(Wref)

            WIni = Wref;%zeros(numel(W),1);%reshape(W',numel(W),1);

            %fprintf('Initial Constraint : ');constraintFunc(WIni)
            %size(WIni)
            %WOpt= fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon)
        %    tic;
            %figure(1);

            bMag = 50;%15;% max(abs(WIni));
            lb = [];%-bMag *  ones(size(WHat));
            ub = [];%bMag * ones(size(WHat));
            options = optimset;
          %  options = optimset(options,'Display', 'iter');
          % options = optimset(options,'Display', 'final');
           % options = optimset(options,'Algorithm', 'active-set');
            options = optimset(options,'MaxFunEvals', 6000);
            options = optimset(options,'Algorithm', 'interior-point');
            
%
           %options = optimset(options,'PlotFcns', {  @optimplotx @optimplotfval @optimplotconstrviolation });
            %options = optimset(options,'PlotFcns', {  @optimplotfval @optimplotconstrviolation });
            [WOpt,FVal,ExitFlag] = fmincon(costFunc,WIni,[],[],[],[],lb,ub,constraintFunc, options);
            %WOpt= fmincon(costFunc,WIni,[],[],[],[],[],[],constraintFunc, options);
       %     toc
            exitFlagMat(i,j) = ExitFlag;

           % [ result ] = testOptimalSolution( WOpt , WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt);
            %save(sprintf('./Data/TetheredMassFourierBasisInput/optim_tMax_%d_basisorder_%d',tspan(2),basisOpt.order),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt');
            %save(sprintf('./Data/TetheredMassFourierBasisInput/optimSigmoidStart_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspan(2)),floor(basisOpt.fourierOrder),floor(10*xDes(1)),floor(10*xDes(2))),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            if(initialTrajectoryType ==1)
                save(sprintf('./Data/SynergyPaper/TetheredMass2D_FourierBasisMDC/MDC_StepStart_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspan(2)),floor(basisOpt.order),floor(10*xDes(1)),floor(10*xDes(2))),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            else
                save(sprintf('./Data/SynergyPaper/TetheredMass2D_FourierBasisMDC/MDC_SigmoidStart_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspan(2)),floor(basisOpt.order),floor(10*xDes(1)),floor(10*xDes(2))),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
            end
            
           % fprintf('Ini :'); disp(WIni');
          %  fprintf('Fin :'); disp(WOpt');
            drawnow();
        end
   end
end

fprintf('Optim Results : \n');

resultMat = zeros(size(exitFlagMat,1)+1,size(exitFlagMat,2)+1);
resultMat(2:end,2:end) = exitFlagMat;
resultMat(1,2:end) = tspanRange;
resultMat(2:end,1) = basisOrderRange;
disp(resultMat);