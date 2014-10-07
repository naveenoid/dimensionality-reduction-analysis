function [ c,ceq ] = reachCartesianConstraint( WHat, desiredPos, tspan,mechSystem, mechOpt,basisFunc,basisOpt)
%REACHCONSTRAINT Constraint on the optimisation to make sure the endpoint
%is reached 
%
%
% Naveen Kuppuswamy (2013) - naveenoid@ifi.uzh.ch

    if(size(WHat,2) == 1) 
		%Its organised as a column, presumably for optimisation, so reshape
		 WPrime = (reshape(WHat,[basisOpt.order+1,mechOpt.inputDim]))';
	else
		WPrime = WHat;
    end
    

    mechBasisSystem = @(t,x,u)mechSystem(t,x,WPrime*u,mechOpt); 
    
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
    
    %tic;
    [t, x] = ode15s(sys,tspan,xi,vopt);
    %[t, x] = ode45(sys,tspan,xi,vopt);
    thetaDotEnd=sys(t(end),x(end,:)');
    xEnd = fkin(x(end,1), x(end,2),mechOpt);
    
    %x(end,1:2);
    ceq = [];
    c(1:2) = norm(xEnd(3:4)' - desiredPos);% - 0.01;%25;%0.01;% - 0.0001;
    c(3)  = abs(x(end,3));% - 0.01; %FINAL velocity joint1
    c(4) = abs(x(end,4));% - 0.01; %FINAL velocity joint2
    %c(5) = abs(thetaDotEnd(3)) - 0.08;%75;%FINAL acceleration joint1
    %c(6) = abs(thetaDotEnd(4)) - 0.08;%075;%FINAL acceleration joint2
    
    %% Works for 0.3, -0.7
%         c(1:2) = norm(xEnd(3:4)' - desiredPos) - 0.025;%0.01;% - 0.0001;
%     c(3) = abs(x(end,3)) - 0.1; %FINAL velocity joint1
%     c(4) = abs(x(end,4)) - 0.1; %FINAL velocity joint2
%     c(5) = abs(thetaDotEnd(3)) - 0.1;%75;%FINAL acceleration joint1
%     c(6) = abs(thetaDotEnd(4)) - 0.1;%075;%FINAL acceleration joint2

    
end




