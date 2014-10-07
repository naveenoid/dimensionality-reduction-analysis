%% Optimising the dimensionality of a Fourier Basis System to follow a set of four waypoints


basisOrderRange = 5;%5;%5:1:8;
tspanRange = 5;%1:0.5:3;%8
endPointRange = 1;%0.6:0.2:1.0;%0.1:0.1:0.5;

for endPoint = endPointRange
   for basisOrder = basisOrderRange 
        for tspanMax = tspanRange
            fprintf('\n-----------------------------------------------------------\n');
            fprintf('TRIAL : optim_tMax_%d_basisorder_%d_target_(%d,%d)',floor(10*tspanMax),floor(basisOrder),floor(10*endPoint),floor(10*endPoint));
            fprintf('\n-----------------------------------------------------------\n');
            
            clearvars -except tspanRange tspanMax basisOrderRange basisOrder endPointRange endPoint
    % 
    % 
    % tspanRange = 1:0.5:8
    % 
    % for tspanMax = tspanRange
    %   %  tspanmax = 3;
    %   clearvars -except tspanMax tspanRange

        % Robot setup
            
        
            xAmp = 0.25;
            yAmp = 0.5;

            ellipseOpt.p1 = [xAmp,0]';
            ellipseOpt.p2 = [0,yAmp]';
            ellipseOpt.p3 = [-xAmp,0]';
            ellipseOpt.p4 = [0,-yAmp]';
            
        
        
            mechOpt = init2DSpringyDampedMass(2);%robotData();
            mechOpt.initialCond = zeros(size(mechOpt.A,1),1);
            %mechOpt.initialCond = [ellipseOpt.p1]
            xi = mechOpt.initialCond;
            tspan = [0;tspanMax];%[0:0.001:10];%train.tspan;
            mechSystem = @sysLTIForced;
            mechOpt.inputDim = size(mechOpt.B,2);
            mechOpt.stateDim = size(mechOpt.A,1);
            mechOpt.outputDim = size(mechOpt.C,1);
            mechOpt.outputs = [1,2];

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
            %xDes = [endPoint,endPoint]';
            %xDes = [0.5 -0.5]';
            %fprintf('xDes\n');
            %disp(xDes);
            
            %FSteady = %computeSteadyStateForce(xDes,mechOpt);

            %tauSteady  = computeSteadyStateTorque(iKin(xDes','up',mechOpt)', mechOpt);
            %fprintf('F Computed : '); FSteady'
            %fprintf('\n');
            % tauSteady = computeSteadyStateTorque(iKin(xDes','down',mechOpt)', mechOpt);
            % fprintf('Tau Computed down : '); tauSteady'
            % fprintf('\n');

            %tauSteady = [1;1];%computeSteadyStateTorque(iKin(xDes','down',mechOpt)', mechOpt);
            %fprintf('Tau Actual : '); tauSteady'
            %fprintf('\n');

            %trainingFunc = @(tTrain)( 1.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
            %trainingFunc = @(tTrain)( [FSteady(1)*ones(1,length(tTrain)) ;(FSteady(2)*((1 + tanh(tTrain-0.5*max(tTrain)))))']);
            trainingFunc = @(tTrain)( 10*[xAmp*sin(((0.5*pi)/tTrain(end))*tTrain)  yAmp*sin(((0.5*pi)/tTrain(end))*tTrain) ]');
            %trainingFunc = @(tTrain)( tauSteady*(1 + tanh(tTrain-max(tTrain)*0.5))')
            %trainingFunc{2} = @(tTrain)( 0.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
            %trainingFunc{3} = @(tTrain)( 1.0*(1 + tanh(tTrain-max(tTrain)*0.5)));
            %trainingFunc{2} = @(tTrain)( sin(pi*(tTrain./(0.5*max(tTrain)))));%@(tTrain)( cos(pi*(tTrain./(max(tTrain)))));
            %trainingFunc{3} = @(tTrain)( sin(pi*(tTrain./(max(tTrain)))));

           % trajectoryName = {'reachStep', 'intrinsic'};
            trajectoryName = {'sinsoidal','intrinsic'};
            result= struct();
            tTrain =(tspan(1):0.05:tspan(2))';

            xD = zeros(2,length(tTrain));

             %   tic;
            fprintf('Learning Training Trajectory\n');
                % trainingData 

            xD = trainingFunc(tTrain);    
            %xD(2,:) = trainingFunc{train}(tTrain);%0.5*(1 + tanh(tTrain-max(tTrain)*0.5));
            %xD(1,:) = trainingFunc{1}(tTrain);
            result.xD = xD;
            %( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
            [WHat,basisOpt] = basisLearning(tTrain,xD,[],[],basisFunc,mechOpt,basisOpt)
            mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 

            mechBasisOpt = mechOpt;
            mechBasisOpt.inputDim = basisOpt.order;
            %uFunc = @(t)WHat*basisFunc(t,basisOpt);

            fprintf('Num of basis funcs : %d\n',basisOpt.order );


            sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

            %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
            %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
            vopt = odeset('AbsTol',1e-1, 'RelTol',5e-2);
            %
            tspanFull = linspace(tspan(1),tspan(2),500);
            fprintf('Simulating time for simple sinsoidal input ');


            fprintf('Starting optimisation\n');
            drawnow;

            fprintf('InitialWeights ');
            WHat
            %[ score, redOrder, redSys, hsv , normHsv] =  evaluateDimCost_NBT(WHat, tspan,mechSystem,mechOpt, basisOpt, redThreshold, cm);
            [score, redOrder , redSys, hsv, normHsv ] = evaluateDimCost_BT_HSV2Cost(WHat,mechOpt,basisOpt,redThreshold,[],[],[],1);

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

            %constraintFunc  = @(WHat)reachConstraint( WHat, desiredTheta, tspan,mechSystem, mechOpt,basisFunc,basisOpt);
                                                        %WIn, mechSystem,mechOpt,tspan, basisFunc,basisOpt, ellipseOpt
            constraintFunc  = @(WHat)ellipseConstraint( WHat,mechSystem,mechOpt,tspan,basisFunc,basisOpt, ellipseOpt);
            %costFunc  = @(WHat)evaluateDimCost_NBT( WHat,tspan,mechSystem, mechOpt, basisOpt, redThreshold, cm,redOrder );
            costFunc = @(WHat)evaluateDimCost_BT_HSV2Cost(WHat,mechOpt,basisOpt,redThreshold,[],[],[],1);
             Wref = reshape(WHat',numel(WHat),1);
            %size(Wref)

            WIni = Wref;%zeros(numel(W),1);%reshape(W',numel(W),1);

            fprintf('Initial Constraint : ');constraintFunc(WIni)
            %size(WIni)
            %WOpt= fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon)
            tic;
            %figure(1);

            lb = -50.0 *  ones(size(WHat));
            ub = 50.0 * ones(size(WHat));
            options = optimset;
            options = optimset(options,'Display', 'iter');
            options = optimset(options,'Algorithm', 'active-set');
            options = optimset(options,'MaxFunEvals', 3000);
            %options = optimset(options,'Algorithm', 'interior-point');

            options = optimset(options,'PlotFcns', {  @optimplotx @optimplotfval @optimplotconstrviolation });
            %options = optimset(options,'PlotFcns', {  @optimplotfval @optimplotconstrviolation });
            [WOpt,FVal,ExitFlag] = fmincon(costFunc,WIni,[],[],[],[],lb,ub,constraintFunc, options);
            %WOpt= fmincon(costFunc,WIni,[],[],[],[],[],[],constraintFunc, options);
            toc

            %close all;
            
            [ result ] = testOptimalSolutionEllipse( WOpt, WIni, xAmp, yAmp,redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt, 'b' )
            %[ result ] = testOptimalSolutionEllipse( WOpt , WIni, redThreshold,  tspan, mechSystem, mechOpt, basisFunc, basisOpt,'b');
            %save(sprintf('./Data/TetheredMassFourierBasisInput/optim_tMax_%d_basisorder_%d',tspan(2),basisOpt.order),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt');
            save(sprintf('./Data/TetheredMassFourierBasisInput/optimEllipse_tMax_%d_basisorder_%d_ellipseAmp_(%d,%d)',floor(10*tspan(2)),floor(basisOpt.fourierOrder),floor(100*xAmp),floor(100*yAmp)),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt','ExitFlag');
        end
   end
end
