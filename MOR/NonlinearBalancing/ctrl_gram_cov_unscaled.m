
function yhat = ctrl_gram_cov_unscaled(OdeFcn,Tspan,ParaVector,Cm,uss,xss,flag) 

% yhat = ctrl_gram_cov_unscaled(OdeFcn,Init,Tspan,ParaVector) 
% ctrl_gram_cov_unscaled computes the controllability gramian for unscaled 
% nonlinear control-affine systems.
% yhat: the controllability gramian
% OdeFcn: the ode function of the system in the form dxdt = f(t,x,u)
% Tspan: [start time, ending time, sampleLength]
% ParaVector: InputNumber p, StateNumber n, OutputNumber k, Orientation Number r and length q
% Cm: CmValue, and s = size(Cm,1)
% uss: the value of input at steady state
% xss: the steady state
% flag == 0, gramian; flag ~= 0, covariance matrix


%global ud

Tol			=	1E-9;

% simulation start time
startTime	=	Tspan(1);
% simulation end time
endTime		=	Tspan(2);
% calculate the time for each sample
SampleLength = Tspan(3);
OutputLength = floor((endTime-startTime)/SampleLength);
% gramian parameters
%basisOpt.order+1 mechOpt.stateDim mechOpt.outputDim 2 100
p = ParaVector(1); % inputOrder
n = ParaVector(2); % stateOrder
k = ParaVector(3); % outputOrder
r = ParaVector(4); % orientation number r = 2
q = ParaVector(5); % not used
cm = Cm;
s = length(Cm);

% initialization for T for controllability

T = [eye(p,p) -eye(p,p)];
e = eye(p,p);
yhat = zeros(n,n);
M = cm;


for l=1:r
for m=1:s
for i=1:p


% initvalue=ones(n,1);
    initvalue = xss;
    
    if flag == 0

        % impulse input
        ud = uss + M(m)*T(1:p,(l-1)*p+1:l*p)*e(:,i)/SampleLength;
        
        OdeFcnMod = @(t,x)OdeFcn(t,x,ud);
        
        [t,y] = ode15s(OdeFcnMod,[startTime startTime+SampleLength],initvalue);
        initvalue = y(length(t),:)';
        ud = uss;
        
        OdeFcnMod = @(t,x)OdeFcn(t,x,ud);

        [t,y] = ode15s(OdeFcnMod,[startTime:SampleLength:endTime],initvalue);

        y = round(y/Tol)*Tol;

        %y = y*diag(xss);

        phi = zeros(n,n);

        %yss = y(length(t),:);
        yss = xss';

        phi = phi + SampleLength*(y-ones(length(t),1)*yss)'*(y-ones(length(t),1)*yss);   %(y(k,:) - yss)'*(y(k,:) - yss);
        yhat = yhat + 1/(r*s*M(m)^2)*phi;
        
    else   %compute controllability covariance matrix
        
        % step input
        ud = uss + M(m)*T(1:p,(l-1)*p+1:l*p)*e(:,i);
        OdeFcnMod = @(t,x)OdeFcn(t,x,ud);
        [t,y] = ode15s(OdeFcnMod,[startTime:SampleLength:endTime],initvalue);
        y = round(y/Tol)*Tol;
        phi = zeros(n,n);
        finalvalue = y(length(t),:);
        phi = phi + SampleLength*(y-ones(length(t),1)*finalvalue)'*(y-ones(length(t),1)*finalvalue);   
        yhat = yhat + 1/(r*s*M(m)^2)*phi;
     
    end

end
end
end

yhat = 0.5*(yhat+yhat');
