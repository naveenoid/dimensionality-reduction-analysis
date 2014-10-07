function [ c,ceq ] = reachConstraint( WHat, desiredTheta, tspan,mechSystem, mechOpt,basisFunc,basisOpt,constraintOpt)
%REACHCONSTRAINT Constraint on the optimisation of dimensionality that the goalstate
%is reached at the end of the movement period when using a controller composed of a weighted summation of a set of basis functions on time.
% PARAM : 
% WHat - Weights 
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
    %thetaDotEnd=sys(t(end),x(end,:)');
   % thetaDotIni = sys(t(1),x(1,:)');
    %xEnd = fkin(x(end,1), x(end,2),mechOpt);
    
    %x(end,1:2);
    ceq = [];
    %c(1:2) = norm(xEnd(3:4)' - desiredPos) - 0.025;%0.01;% - 0.0001;
    c(1) = norm(x(end,1:2) - desiredTheta);% - 1e-6;
  %  c(2) = abs(x(end,3)) -  - 1e-2; %FINAL velocity joint1
  %  c(3) = abs(x(end,4))- 1e-2; %FINAL velocity joint2
 %   c(5) = abs(thetaDotEnd(3)) - 1e-2;%75;%FINAL acceleration joint1
 %   c(6) = abs(thetaDotEnd(4)) - 1e-2;%075;%FINAL acceleration joint2
  %  c(5) = abs(thetaDotIni(1)) - 1e-3; % INITIAL velocity joint1
  %  c(6) = abs(thetaDotIni(2)) - 1e-3; % INITIAL velocity joint2
    c(2) = abs(x(end,3)) - 1e-2; % FINAL velocity joint1
    c(3) = abs(x(end,4)) - 1e-2; % FINAL velocity joint2
%c(9) = abs(thetaDotIni(3)) - 1e-4; % INITIAL velocity joint1
   % c(10) = abs(thetaDotIni(4)) - 1e-4; % INITIAL velocity joint2
  %  c(9) = abs(thetaDotIni(3)) - 10.009; % INITIAL Acceleration joint1
  %  c(10) = abs(thetaDotIni(4)) - 10.009; % INITIAL Acceleration joint2
    
    
end




