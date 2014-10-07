
function yhat = ctrl_gram_cov(OdeFcn,Tspan,ParaVector,Cm,flag) 

% yhat = ctrl_gram_cov(OdeFcn,Tspan,ParaVector,Cm,flag) 
% ctrl_gram_cov computes the controllability gramian or covariance matrix for nonlinear systems.
% yhat: the controllability gramian (flag == 0) or covariance matrix (flag ~= 0)
% OdeFcn: the ode function of the system in the form dxdt = f(t,x,u)
% Tspan: [start time, ending time, sampleLength]
% ParaVector: InputNumber p, StateNumber n, OutputNumber k, Orientation Number r and length q
% Cm: CmValue, and s = size(Cm,1) 
% flag == 0, gramian; flag ~= 0, covariance matrix


%global ud
Tol			=	1E-9;

% simulation start time
startTime	=	Tspan(1);
% simulation end time
endTime		    = Tspan(2);
SampleLength    = Tspan(3);
OutputLength    = floor((endTime-startTime)/SampleLength);
% parameters
p = ParaVector(1);
n = ParaVector(2);
k = ParaVector(3);
r = ParaVector(4);
q = ParaVector(5);
cm = Cm;
s = length(Cm);

% initialization
T = [eye(p,p) -eye(p,p)];
e = eye(p,p);
yhat = zeros(n,n);
M = cm;

for l=1:r
for m=1:s
for i=1:p

    initvalue=ones(n,1);
    
    if flag == 0,  % compute controllability gramian
        % impulse input
        ud = ones(p,1) + M(m)*T(1:p,(l-1)*p+1:l*p)*e(:,i)/SampleLength;
        OdeFcnMod = @(t,x)OdeFcn(t,x,ud);
        [t,y] = ode15s(OdeFcnMod,[startTime startTime+SampleLength],initvalue);
        initvalue = y(length(t),:)';
        ud = ones(p,1);
        OdeFcnMod = @(t,x)OdeFcn(t,x,ud);
        [t,y] = ode15s(OdeFcnMod,[startTime:SampleLength:endTime],initvalue);
        y = round(y/Tol)*Tol;
        phi = zeros(n,n);
        initvalue = ones(n,1);
        phi = phi + SampleLength*(y-ones(length(t),1)*initvalue')'*(y-ones(length(t),1)*initvalue');   
        yhat = yhat + 1/(r*s*M(m)^2)*phi;
        
    else   % compute controllability covariance matrix
        % step input
        ud = ones(p,1) + M(m)*T(1:p,(l-1)*p+1:l*p)*e(:,i);
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
