function [ c,ceq ] = reachConstraint( WHat, desiredPos, tspan,mechSystem, mechOpt,basisFunc,basisOpt)
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
    vopt = odeset( 'AbsTol',1e-1,'RelTol',5e-2);%1e-5, 'RelTol',1e-6);
    %
   % vopt = odeset('InitialStep',1e-10);
   % tspanFull = linspace(tspan(1),tspan(2), 500);
    xi = mechOpt.initialCond;
    %
    
    %tic;
    [t, x] = ode15s(sys,tspan,xi,vopt);
    %[t, x] = ode45(sys,tspan,xi,vopt);
    
    xEnd = fkin(x(end,1), x(end,2),mechOpt);
    
    %x(end,1:2);
    ceq = [];
    c = norm(xEnd(3:4)' - desiredPos) - 0.01;%0.01;% - 0.0001;
end

