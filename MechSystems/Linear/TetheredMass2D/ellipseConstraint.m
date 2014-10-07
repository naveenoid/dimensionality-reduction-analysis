function [ c,ceq ] = ellipseConstraint(WIn, mechSystem,mechOpt,tspan, basisFunc,basisOpt, ellipseOpt)
%ELLIPSECONSTRAINT Summary of this function goes here
%   Detailed explanation goes here
% 
% function [ c,ceq ] = reachConstraintzeroAccln( WIn, initialPos,desiredPos,trainingTSpan, basisFunc,polyOpt, basisSys, mode )
% %REACHCONSTRAINT Summary of this function goes here
% %   Detailed explanation goes here



if(size(WIn,2) == 1) 
    %Its organised as a column, presumably for optimisation, so 		reshape
     W = (reshape(WIn,numel(WIn)/2,2))';
else
    W = WIn;
end


    mechBasisSystem = @(t,x,u)mechSystem(t,x,W*u,mechOpt); 
    
    mechBasisOpt = mechOpt;
    mechBasisOpt.inputDim = basisOpt.order+1;
    %uFunc = @(t)WHat*basisFunc(t,basisOpt);

    sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

    %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
    %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
   % vopt = odeset('InitialStep',1e-10,'MaxStep',2e-2, 'AbsTol',1, 'RelTol',1);
    vopt = odeset( 'AbsTol',5e-2,'RelTol',1e-3);%1e-5, 'RelTol',1e-6);
    %
   % vopt = odeset('InitialStep',1e-10);
   % tspanFull = linspace(tspan(1),tspan(2), 500);
    xi = mechOpt.initialCond;
    %
    testingTSpan = tspan(1):0.01: 10*tspan(end);
    %tic;
    [t, xOut] = ode15s(sys,testingTSpan,xi,vopt);
% 
% % integrate and provide result
% basisSys.B = basisSys.B * W;    
% basisSys.mechSys =  @(t,x,u)sysLTIInputFed(t,x,u,basisSys);
% sys = @(t,x)sysPolyDInput(t,x,polyOpt,basisSys);%;,DStar);
% testingTSpan = trainingTSpan;
% testingTSpan = trainingTSpan(1):0.01: 3*trainingTSpan(end);
% % vopt = odeset ('InitialStep',1e-2,'MaxStep',5e-1);
% % tic;
% [tOut, xOut] = ode45(sys,testingTSpan,initialPos);%,vopt);
% %toc

%testingTSpan = 2*trainingTSpan(end):0.01:3*trainingTSpan(end);
testingPts = find(testingTSpan>(6*tspan(end)));


%minDistance = @(pt,vect,ptFrom)norm(vect(pt,:)-ptFrom);
%minDistance1 = @(pt)minDistance(pt,xOut(:,1:2),ellipseOpt.p1);
%minDistance2 = @(pt)minDistance(pt,xOut(:,1:2),ellipseOpt.p2);
%minDistance3 = @(pt)minDistance(pt,xOut(:,1:2),ellipseOpt.p3);
%minDistance4 = @(pt)minDistance(pt,xOut(:,1:2),ellipseOpt.p4);

% figure(1);
% plot(t,xOut(:,1),'r',t,xOut(:,2),'b');
% xlabel('time t(sec)');
% ylabel('position x,y(m)');
% figure(2);
% 
% plot(xOut(:,1),xOut(:,2)); hold on;
% plot(xOut(testingPts,1),xOut(testingPts,2),'b');
% plot(ellipseOpt.p1(1),ellipseOpt.p1(2),'ko');plot(ellipseOpt.p2(1),ellipseOpt.p2(2),'ko');
% plot(ellipseOpt.p3(1),ellipseOpt.p3(2),'ko');plot(ellipseOpt.p4(1),ellipseOpt.p4(2),'ko');
% xlabel('time t(sec)');
% ylabel('position x,y(m)');


c(1) = norm(min((xOut(testingPts,1:2) - repmat(ellipseOpt.p1',length(testingPts),1)).^2));%fminbnd(minDistance1,testingPts(1),testingPts(end));
c(2) = norm(min((xOut(testingPts,1:2) - repmat(ellipseOpt.p2',length(testingPts),1)).^2));%fminbnd(minDistance2,testingPts(1),testingPts(end));
c(3) = norm(min((xOut(testingPts,1:2) - repmat(ellipseOpt.p3',length(testingPts),1)).^2));%fminbnd(minDistance3,testingPts(1),testingPts(end));
c(4) = norm(min((xOut(testingPts,1:2) - repmat(ellipseOpt.p4',length(testingPts),1)).^2));%fminbnd(minDistance4,testingPts(1),testingPts(end));

%  c(1) = minDistance(ellipseOpt.xMin,xOut(testingPts,1));
%  c(2) = minDistance(ellipseOpt.xMin,xOut(testingPts,1));
%  c(3) = minDistance(ellipseOpt.xMin,xOut(testingPts,1));
%  c(4) = minDistance(ellipseOpt.xMin,xOut(testingPts,1));

% figure(1); clf;
% figure(2); clf;



ceq = [];
%c = norm(xOut(end,:)' - desiredPos)+norm(sys(testingTSpan(end),xOut(end,:))' -[0,0,0,0]')  - 0.0001;
%xOut(end,:) 


