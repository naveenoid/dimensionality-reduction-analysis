function [ c,ceq ] = reachConstraint( WIn, desiredPos,mechSystem,mechOpt,tspan, basisFunc,basisOpt )
%REACHCONSTRAINT Summary of this function goes here
%   Detailed explanation goes here


 
	if(size(WIn,2) == 1) 
		%Its organised as a column, presumably for optimisation, so 		reshape
		 WPrime = (reshape(WIn,numel(WIn)/2,2))';
	else
		WPrime = WIn;
    end
    

    mechBasisSystem = @(t,x,u)mechSystem(t,x,WPrime*u,mechOpt); 
    
    mechBasisOpt = mechOpt;
    mechBasisOpt.inputDim = basisOpt.order+1;
    %uFunc = @(t)WHat*basisFunc(t,basisOpt);

    sys = @(t,x)mechBasisSystem(t,x,basisFunc(t,basisOpt));

    %vopt = odeset ('InitialStep',1e-10,'MaxStep',1e-5);
    %vopt = odeset('InitialStep',1e-10,'MaxStep',1e-4, 'AbsTol',1, 'RelTol',1);
   % vopt = odeset('InitialStep',1e-10,'MaxStep',2e-2, 'AbsTol',1, 'RelTol',1);
    vopt = odeset( 'AbsTol',5e-3,'RelTol',1e-4);
    %odeset( 'AbsTol',5e-2,'RelTol',1e-3);%1e-5, 'RelTol',1e-6);
    %
   % vopt = odeset('InitialStep',1e-10);
   % tspanFull = linspace(tspan(1),tspan(2), 500);
    xi = mechOpt.initialCond;
    %
    
    %tic;
    [t, x] = ode15s(sys,tspan,xi,vopt);
    %[t, x] = ode45(sys,tspan,xi,vopt);
    xEnd=mechOpt.C*x(end,:)';
    %xEnd = fkin(x(end,1), x(end,2),mechOpt);
 %   xEndVel = sys(t(end),x(end,:)');
   % xStartVel = sys(t(1),x(1,:)');
    %xVel = diff(mechOpt.C*x')./diff(t);
    
    %x(end,1:2);
    ceq = [];
    %c(1:2) = norm(xEnd(3:4)' - desiredPos) - 0.025;%0.01;% - 0.0001;
    c(1) = norm(desiredPos-xEnd);% - 5e-3;% - 1e-5;% - 1e-10;
    c(2) = abs(x(end,3));%(x(end,3)).^2;%- 1e-5;% - 1e-3; %Final Velocity
    c(3) = abs(x(end,4));%(x(end,4)).^2;%- 1e-5; % - 1e-3; %Final Velocity
    %c(2) = abs(x<End(3)) - 1e-2;% - 1e-12; %Final Velocity
    %c(3) = abs(xEnd(4)) - 1e-2;% - 1e-12;%Final Velocity
%    c(2) = abs(xEndVel(3));% - 1e-12; %Final Acceleration
%    c(3) = abs(xEndVel(4));% - 1e-12;%Final Acceleration
 %   c(5) = abs(xStartVel(1));% - 1e-4; %Initial Velocity
 %   c(6) = abs(xStartVel(2));% - 1e-4; %Initial Velocity       
    %c(9) = abs(xStartVel(3)) - 1e-4; %Initial Acceleration
    %c(10) = abs(xStartVel(4)) - 1e-4; %Initial Acceleration    
    

    %xOut(end,:) 
%end

%ceq = norm((inputSys.A * pinv(inputSys.C) * desiredPos) + (inputSys.B*DPrime*basisFunc(finalTime)));

end

