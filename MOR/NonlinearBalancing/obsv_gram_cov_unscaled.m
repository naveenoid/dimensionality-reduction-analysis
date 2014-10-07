
function xhat = obsv_gram_cov_unscaled(OdeFcn,Tspan,ParaVector,Cm,OutputIndex,uss,xss,flag) 

% xhat = obsv_gram_cov_unscaled(OdeFcn,Tspan,ParaVector,OuptputIndex,uss,xss) 
% obsv_gram_cov_unscaled computes the observability gramian for unscaled 
% nonlinear control-affine systems.
% xhat: the controllability gramian
% OdeFcn: the ode function of the system in the form dxdt = f(t,x,u)
% Tspan: [start time, ending time, sampleLength]
% ParaVector: InputNumber p, StateNumber n, OutputNumber k, Orientation Number r and length q
% Cm: CmValue, and s = size(Cm,1) 
% OutputIndex: the indices of outputs corresponding to states
% uss: the value of input at steady state
% xss: steady state
% flag == 0, gramian; flag ~= 0, covariance matrix

global ud 

Tol			=	1E-9;

% simulation start time
startTime	=	Tspan(1);
% simulation end time
endTime		=	Tspan(2);
% calculate the time for each sample
SampleLength = Tspan(3);
OutputLength = floor((endTime-startTime)/SampleLength)+1;
% gramian parameters
%basisOpt.order+1 mechOpt.stateDim mechOpt.outputDim 2 100
p = ParaVector(1); % inputOrder
n = ParaVector(2); % stateOrder
k = ParaVector(3); % outputOrder
r = ParaVector(4); % orientation number r = 2
q = ParaVector(5); % not used
cm = Cm;
s = length(cm);

% initialization for T for controllability

T = [eye(n,n) -eye(n,n)];
e = eye(n,n);
xhat = zeros(n,n);
M = cm;


for l=1:r
    for m=1:s
        chsi = zeros(n,n); 
        z = zeros(n,OutputLength*k);   
        for i=1:n
        
            initvalue = xss;
            % apply perturbed initial condition
            initvalue = initvalue + M(m)*T(1:n,(l-1)*n+1:l*n)*e(:,i);
            ud = uss;

            OdeFcnMod = @(t,x)OdeFcn(t,x,ud);            
            [t,y] = ode15s(OdeFcnMod,[startTime:SampleLength:endTime],initvalue);
            y = round(y/Tol)*Tol;
            
            initvalue = xss;

            if flag == 0,   % compute observability gramian
            
                for iii = 1:k,
                    z(i,OutputLength*(iii-1)+1:OutputLength*iii) = (y(:,OutputIndex(iii)) - ones(length(t),1)*initvalue(OutputIndex(iii)))';
                end
                
            else   % compute observability covariance matrix
        
                finalvalue = y(length(t),:)';
                for iii = 1:k,
                    z(i,OutputLength*(iii-1)+1:OutputLength*iii) = xss(OutputIndex(iii))*(y(:,OutputIndex(iii)) - ones(length(t),1)*finalvalue(OutputIndex(iii)))';
                end
            end
        end                
                
        chsi = z*z';
        xhat = xhat + 1/(r*s*M(m)^2)*SampleLength*T(1:n,(l-1)*n+1:l*n)*chsi*T(1:n,(l-1)*n+1:l*n)';
    end
end
xhat = 0.5*(xhat+xhat');
