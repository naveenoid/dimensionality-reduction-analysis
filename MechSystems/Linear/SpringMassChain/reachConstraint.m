function [ c,ceq ] = reachConstraint( WIn, desiredPos,mechSystem,mechOpt,tspan, basisFunc,basisOpt )
%REACHCONSTRAINT Summary of this function goes here
%   Detailed explanation goes here

  %  sw = size(WIn,2);
 
	if size(WIn,2) == 1 %&& mod(size(WIn,1),2) == 0  
		%Its organised as a column, presumably for optimisation, so 		reshape
		 WPrime = (reshape(WIn,[basisOpt.order+1,size(mechOpt.B,2)]))';%(reshape(WIn,numel(WIn)/2,2))';
	else
		WPrime = WIn;
    end    

% if mode == 1 
%     
%     % linear constraints at final time
%    % tic;
% 
%     tfinal = trainingTSpan(end);
%     Aeq = @(tf) [tf.^(0:polyOpt.order), zeros(1,polyOpt.order+1);
%        zeros(1,polyOpt.order+1), tf.^(0:polyOpt.order)
%       ];
%      xDotF = [desiredPos(3:4); 0;0];
%     xF = desiredPos;
%      %xF = [trajectoryOpt.xf; trajectoryOpt.yf; trajectoryOpt.xDotf; trajectoryOpt.yDotf];
%   
%     Beq = pinv(basisSys.B)*(xDotF-basisSys.A*xF);
%    % WeqT = pinv(basisSys.B)*[pinv(basisSys.C), -basisSys.A*pinv(basisSys.C)]*WStar;
%     Weq = reshape(W',numel(W),1);
%     basisFunc = @(t)[t.^(0:polyOpt.order)]';
% 
%     ceq = [];
%     c = norm(Beq - Aeq(tfinal) * Weq) - 0.0001; 
%    % toc
% else
    % integrate and provide result
%     basisSys.B = basisSys.B * W;    
%     basisSys.mechSys =  @(t,x,u)sysLTIInputFed(t,x,u,basisSys);
%     sys = @(t,x)sysPolyDInput(t,x,polyOpt,basisSys);%;,DStar);
%     
%     mechBasisSystem = @(t,x,u)mechSystem(t,x,WHat*u,mechOpt); 
% 
%     mechBasisOpt = mechOpt;
%     mechBasisOpt.inputDim = basisOpt.order+1;
%     %uFunc = @(t)WHat*basisFunc(t,basisOpt);
% 
% 
%     sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));
%     
%    testingTSpan = trainingTSpan;
%    testingTSpan = trainingTSpan(1):0.01: trainingTSpan(end);
%    % vopt = odeset ('InitialStep',1e-2,'MaxStep',5e-1);
%    % tic;
%     [tOut, xOut] = ode45(sys,testingTSpan,initialPos);%,vopt);
%     %toc
%     ceq = [];
%     c = norm(xOut(end,:)' - desiredPos) - 0.0001;

    mechBasisSystem = @(t,x,u)mechSystem(t,x,WPrime*u,mechOpt); 
    
    mechBasisOpt = mechOpt;
    mechBasisOpt.inputDim = basisOpt.order+1;
    %uFunc = @(t)WHat*basisFunc(t,basisOpt);

    sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

    %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
    %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
   % vopt = odeset('InitialStep',1e-10,'MaxStep',2e-2, 'AbsTol',1, 'RelTol',1);
    
   %vopt = odeset( 'AbsTol',5e-2,'RelTol',1e-3);%1e-5, 'RelTol',1e-6);
   vopt = odeset( 'AbsTol',1e-3,'RelTol',1e-4);%1e-5, 'RelTol',1e-6);
    %
   % vopt = odeset('InitialStep',1e-10);
   % tspanFull = linspace(tspan(1),tspan(2), 500);
    xi = mechOpt.initialCond;
    %
    
    %tic;
    [t, x] = ode15s(sys,tspan,xi,vopt);
    %[t, x] = ode45(sys,tspan,xi,vopt);
    xEnd=mechOpt.C*x(end,:)';
    
    
   % xEndV = sys(t(end),x(end,:)');
   % xEndVel = xEndV(end);
   % xStartV = sys(t(1),x(1,:)');
   % xStartVel = xStartV(end);
   
   %xVel = diff(mechOpt.C*x')./diff(t);
    
    %x(end,1:2);
    ceq = [];
    c(1) = norm(xEnd - desiredPos);% - 1e-10;
    c(2) = abs(x(end,end)); %- 1e-12; %Final Velocity
    c(3) = abs(x(1,end));% - 1e-12; %Initial Velocity
   % c(6) = abs(xStartVel(2)) - 1e-12; %Initial Velocity       
    %c(9) = abs(xStartVel(3)) - 1e-4; %Initial Acceleration
    %c(10) = abs(xStartVel(4)) - 1e-4; %Initial Acceleration    
    

    %xOut(end,:) 
%end

%ceq = norm((inputSys.A * pinv(inputSys.C) * desiredPos) + (inputSys.B*DPrime*basisFunc(finalTime)));

end

