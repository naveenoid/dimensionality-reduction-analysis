
function xhat = obsv_gram_cov(OdeFcn,Tspan,ParaVector,Cm,OutputIndex,xss,flag) 

% xhat = obsv_gram_cov(OdeFcn,Tspan,ParaVector,OuptputIndex,uss,xss) 
% obsv_gram_cov computes the observability gramian or covariance matrix for nonlinear control-affine systems.
% xhat: the controllability gramian or covariance matrix
% OdeFcn: the ode function of the system  in the form dxdt = f(t,x,u)
% Tspan: [start time, ending time, sampleLength]
% ParaVector: InputNumber p, StateNumber n, OutputNumber k, Orientation Number r and length q
% Cm: CmValue, and s = size(Cm,1) 
% OutputIndex: the indices of outputs corresponding to states
% xss: steady state
% flag == 0, gramian; flag ~= 0, covariance matrix


%global ud 

Tol			=	1E-9;

% simulation start time
startTime	=	Tspan(1);
% simulation end time
endTime		=	Tspan(2);
SampleLength = Tspan(3);
OutputLength = floor((endTime-startTime)/SampleLength)+1;
% parameters
p = ParaVector(1);
n = ParaVector(2);
k = ParaVector(3);
r = ParaVector(4);
q = ParaVector(5);
cm = Cm;
s = length(Cm);

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
    
            initvalue = ones(n,1);
    
            % apply perturbed initial condition
            initvalue = initvalue + M(m)*T(1:n,(l-1)*n+1:l*n)*e(:,i);
            ud = ones(p,1);
            
            OdeFcnMod = @(t,x)OdeFcn(x,ud);
            
            [t,y] = ode15s(OdeFcnMod,[startTime:SampleLength:endTime],initvalue);
            y = round(y/Tol)*Tol;
            initvalue = ones(n,1);
    
            if flag == 0,  % compute observability gramian
    
                for iii = 1:k,
                    z(i,OutputLength*(iii-1)+1:OutputLength*iii) = xss(OutputIndex(iii))*(y(:,OutputIndex(iii)) - ones(length(t),1)*initvalue(OutputIndex(iii)))';
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
