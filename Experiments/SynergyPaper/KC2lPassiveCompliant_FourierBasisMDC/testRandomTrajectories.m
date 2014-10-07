%Script to test randomly generated trajectories for dimensionality on the compliantKinematicChainSystem


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
mechOpt.stateDim = 10;
mechOpt.outputDim = 2;
mechOpt.outputs = [1,2];

%polyBasis setup;
basisFunc = @polyBasis;
basisLearning = @polyBasisWeightLearning;
basisOpt.order = 8;

testMax = 10;
result= struct();
tTrain =(tspan(1):0.1:tspan(2))';

randCartesianTarget(1) = 0.4;
randCartesianTarget(2) = -0.6;

invKin

randTarget(1) 


accuracyRadius = 0.05;

RandomWayPointVector = []


train = 0;

while(train<testMax)
    
    tic;
    fprintf('Acquiring Random Trajectory %d\n',train);
    % trainingData 

    %xD = %trainingFunc{train}(tTrain);%0.5*(1 + tanh(tTrain-max(tTrain)*0.5));

    %result(train).xD = xD;
    
    numpts = polyOpt.order*2;
    num2 = numpts/2+4;
    %p1pts = 0.5 -1*rand(numpts,1);
    %p2pts =  0.5 -1*rand(numpts,1);
    %r1pts = 0.2 -1*rand(num2,1);
    %r2pts = 0.7 -1*rand(num2,1);

    r1pts = 3*rand(num2,1) - 2;
    r2pts = 3*rand(num2,1) - 2;

    p1pts = [r1pts;xd*ones(numpts-num2,1)];
    p2pts =  [r2pts;yd*ones(numpts-num2,1)];

    tpts = [linspace(0,10,numpts)]';
    polyOpt.poly(1,:) = fliplr(polyfit(tpts,p1pts,polyOpt.order));
    polyOpt.poly(2,:) = fliplr(polyfit(tpts,p2pts,polyOpt.order));

    
    
    %( t, xD, xDdot, fInvMech, fBasis, mechOpt,basisOpt )
   % WHat = 0.2*(rand(2,basisOpt.order+1) - 0.5);%(basisLearning(tTrain,xD,[],[],basisFunc,mechOpt,basisOpt)
   

    mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
    %uFunc = @(t)WHat*basisFunc(t,basisOpt);

    sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

    %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
    %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
    vopt = odeset('InitialStep',1e-10,'MaxStep',1e-2, 'AbsTol',1, 'RelTol',1);
    %
    [t, x] = ode15s(sys,tspan,xi,vopt);
    
    disp(WHat);
    endPt = fkin(x(end,1),x(end,2),robotData());
    fprintf('Target Reached : ');
    trajectoryTrace_snapshot(fkin(x(:,1),x(:,2),robotData()),t,robotData(),5,0);
    disp(endPt(3:4));
    distTarget = norm(randTarget' - endPt(3:4)');
    if(distTarget > accuracyRadius); 
        continue;
    end
    fprintf('Target Accepted for analysis');
    
    result(train).WHat = WHat;
    result(train).t = t;
    result(train).x = x;
%     trajectoryTrace_snapshot(fkin(x(:,1),x(:,2),robotData()),t,robotData(),4,0);
% 
%     pt = figure(1);
%     title('Position Trajectory');
%     subplot(2,1,1);
%     plot(t,x(:,1:2)); hold on;
%     ylabel('Position');
%     xlabel('time t(secs)');
%     legend('q_1','q_2');
% 
%     subplot(2,1,2);
%     %title('inputs computed by polybasis');
%     plot(t,WHat*basisFunc(t,basisOpt), tTrain,xD,'k'); 

    fprintf('Empirical Reduction of Trajectory %d\n',train);
    drawnow;

    %% Checking dimensionality using empirical gramians

    uss = zeros(basisOpt.order+1,1);
    xss = zeros(mechOpt.stateDim,1);


    Wc3 = ctrl_gram_cov_unscaled(mechBasisSystem,[0 50 0.05],[basisOpt.order+1 mechOpt.stateDim mechOpt.outputDim 2 2000], 0.5, uss, xss,0);
    Wo3 = obsv_gram_cov_unscaled(mechBasisSystem,[0 50 0.05],[basisOpt.order+1 mechOpt.stateDim mechOpt.outputDim 2 2000], 0.5, mechOpt.outputs,uss,xss,0);

    [Trans, invTrans, Wcb3, Wob3, svd_Wcb3, svd_Wob3] = bal_realization(Wc3,Wo3,10) ;

    %fprintf('\n svd controllability * observability\n');
    hsv = ((svd_Wcb3.*svd_Wob3).^0.5)';
    
    result(train).hsv = hsv;
    result(train).Trans = Trans;
    result(train).invTrans = invTrans;
% 
%     plot(1:10,cumsum(hsv)./sum(hsv),'-o'); hold on;
%     axis tight;
%     %title('Hankel Singular Values');
%     xlabel('State');
%     ylabel('Normalised HSV');

train = train+1;
toc
end
cols = {'r','b','g'};


for train = 1:3
   
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
    
    figure(3);
        plot(1:10,cumsum(result(train).hsv)./sum(result(train).hsv),strcat(cols{train},'-o')); hold on;
        axis tight;
        title('Hankel Singular Values');
        xlabel('State');
        ylabel('Normalised HSV');
end
%figure(3);
%title('Training Trajectory Provided');
%plot(tTrain,xD)
% 
% figure(2);
% %subplot(2,1,2);
% %title('Velocity Trajectory');
% plot(t,x(:,3:4)); hold on;
% ylabel('Velocity');
% xlabel('time t(secs)');
% legend('dq_1','dq_2');
% 
% figure(3);
% %title('Compliant Actuation');
% subplot(2,1,1)  
% plot(t,x(:,5:6)); hold on;
% xlabel('time t(secs)');
% ylabel('Muscle Lengths');
% 
% legend('l_{m1,2}','l_{m3,4}');
% %figure;
% %title('Muscle Activation');
% subplot(2,1,2)
% plot(t,x(:,9:10));xlabel('time t(secs)'); hold on;
% ylabel('Muscle Activations');
% legend('\alpha_{m1,2}');
% %legend('shoulder','elbow');
% %armMotion(fkin(x(:,1),x(:,2),robotData()),t,robotData());
% trajectoryTrace_snapshot(fkin(x(:,1),x(:,2),robotData()),t,robotData(),4,0);
% 
% 
% Trans = result(d).Trans;
% invTrans = result(d).invTrans;
% hsv = result(d).hsv;
% red_n = 5;
% n= 10;
% 
% 
% sys = @(t,x)planar2dofArm_red(t,x, ud, robot, Trans, invTrans, n,red_n);
% 
% 
% vopt = odeset('InitialStep',1e-10,'MaxStep',1e-3, 'AbsTol',1e-2, 'RelTol',1e-2);
% 
% cumsumHSV = cumsum(hsv)./(sum(hsv));
% sum(cumsumHSV<0.9);
% 
% figure(5);
% plot(1:10,cumsumHSV,'-o');
% hold on;
% %plot(1:10,0.9*ones(1,10),'k');
% xlabel('State');
% ylabel('Normalised HSV');
% 
% 
% xRedInterp = zeros(2,length(tInterp));
% fprintf('Testing reduced order models \n'); drawnow();
% redArray = 3;%3:1:11;
% 
% fprintf('Red Order : %d\n',red_n);
%     %
% [tred, xtemp] = ode15s(sys,tspan,Trans*xi,vopt);
% %[t, x] = ode45(sys,tspan,xi,vopt);
% 
% xred = (invTrans*xtemp')';
% %  if(tred(end) == t(end))
%    xRedInterp(1,:) = interp1(tred,xred(:,1)',tInterp);
%    xRedInterp(2,:) = interp1(tred,xred(:,2)',tInterp);
%    errorStore(d,:) = norm(xRedInterp(:,:)-xInterp(:,:));
%    errorEndStore(d,:) = norm(xRedInterp(:,end)-xInterp(:,end));
% 
%    fprintf('xend : '); disp(xRedInterp(:,end));
%    fprintf('xIntrep : '); disp(xInterp(:,end));
% %  end
% %% Plotting
% 
% figure(pt);
% hold on;
% hold on;plot(tred,xred(:,1:2),'--');
% 
% 
% trajectoryTrace_snapshot(fkin(xred(:,1),xred(:,2),robotData()),tred,robotData(),4,1);
% figure(4); grid on;
% 
%     %end
% 
% %ylabel('Position');
% %figure(pt+1);
% %hold on;plot(tred,xred(:,3:4),'--');
% 
% %figure(4);print('-djpeg100','-r200','./plots/hsv_fig');
% 
% %figure(1);print('-djpeg100','-r200','./plots/positionTrajectory');
% %figure(2);print('-djpeg100','-r200','./plots/velocityTrajectory');
% %figure(3);print('-djpeg100','-r200','./plots/muscleActivation');
% %figure(4);print('-djpeg100','-r200','./plots/robotMotion');
% 
% figureFolder = './Plots/CompliantKinematicChainBasisInputReduce/';
% 
% %mkdir(strcat(figureFolder,sprintf('multiDamping_%d',red_n)));
% 
% figure(1);print('-depsc2','-r200',strcat(figureFolder,sprintf('multiDamping_%d/positionTrajectory',red_n)));
% figure(2);print('-depsc2','-r200',strcat(figureFolder,sprintf('multiDamping_%d/velocityTrajectory',red_n)));
% figure(3);print('-depsc2','-r200',strcat(figureFolder,sprintf('multiDamping_%d/muscleActivation',red_n)));
% figure(4);print('-depsc2','-r200',strcat(figureFolder,sprintf('multiDamping_%d/robotMotion',red_n)));
% figure(5);print('-depsc2','-r200',strcat(figureFolder,sprintf('multiDamping_%d/hsvDamping',red_n)));
% 
% %trajectoryTrace_snapshot(fkin(x(:,1),x(:,2),robotData()),tred,robotData(),5,0);
% 
% 
% 
% idx= figure;
% plot(dampingRange,errorStore./max(errorStore),'-o',dampingRange,errorEndStore./max(errorEndStore),'r-o');%redArray([1,3:9]),errorStore([1,3:9],:),'-o');
% grid on;
% axis tight;
% xlabel('Damping');
% ylabel('Normalised Error');
% legend('Trajectory Error','End Position Error');
% figure(idx);print('-depsc2','-r200',strcat(figureFolder,sprintf('multiDamping_%d/errorWrtDamping', red_n)));