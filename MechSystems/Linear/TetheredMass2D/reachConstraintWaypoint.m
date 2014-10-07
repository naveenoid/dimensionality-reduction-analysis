function [ c,ceq ] = reachConstraint( WIn, trajectoryOpt,trainingTSpan, basisFunc,polyOpt, basisSys, mode )
%REACHCONSTRAINT Summary of this function goes here
%   Detailed explanation goes here

    
	if(size(WIn,2) == 1) 
		%Its organised as a column, presumably for optimisation, so 		reshape
		 W = (reshape(WIn,numel(WIn)/2,2))';
	else
		W = WIn;
    end

    initialPos = [trajectoryOpt.x0;trajectoryOpt.y0;trajectoryOpt.xDot0;trajectoryOpt.yDot0];
    desiredPos = [trajectoryOpt.xf;trajectoryOpt.yf;trajectoryOpt.xDotf;trajectoryOpt.yDotf];
    
if mode == 1 
    %% To be debugged - technique not working
    % linear constraints at final time
   % tic;

    tfinal = trainingTSpan(end);
    Aeq = @(tf) [tf.^(0:polyOpt.order), zeros(1,polyOpt.order+1);
       zeros(1,polyOpt.order+1), tf.^(0:polyOpt.order)
      ];
     xDotF = [desiredPos(3:4); 0;0];
    xF = desiredPos;
     %xF = [trajectoryOpt.xf; trajectoryOpt.yf; trajectoryOpt.xDotf; trajectoryOpt.yDotf];
  
    Beq = pinv(basisSys.B)*(xDotF-basisSys.A*xF);
   % WeqT = pinv(basisSys.B)*[pinv(basisSys.C), -basisSys.A*pinv(basisSys.C)]*WStar;
    Weq = reshape(W',numel(W),1);
    basisFunc = @(t)[t.^(0:polyOpt.order)]';

    ceq = [];
    c = norm(Beq - Aeq(tfinal) * Weq) - 0.0001; 


else
    % integrate and provide result
    basisSys.B = basisSys.B * W;    
    basisSys.mechSys =  @(t,x,u)sysLTIInputFed(t,x,u,basisSys);
    sys = @(t,x)sysPolyDInput(t,x,polyOpt,basisSys);%;,DStar);
   testingTSpan = trainingTSpan;
   testingTSpan = trainingTSpan(1):0.01: trainingTSpan(end);
   % vopt = odeset ('InitialStep',1e-2,'MaxStep',5e-1);
   % tic;
    [tOut, xOut] = ode45(sys,testingTSpan,initialPos);%,vopt);
    %toc
    ceq = [];
    
    c = zeros(trajectoryOpt.numWayPoints+1,1);
    
    c(1) = norm(xOut(end,:)' - desiredPos) - 0.0001;


	%%%%%%%%
    
%    function [tPoint] = getTPoint(t,tOut)
 %       tVect = find(t>=tOut);
  %      tPoint = tVect(1);
  %  end
    


	for i = 1:trajectoryOpt.numWayPoints	
	    xDesiredWayPt = [trajectoryOpt.xWayPoint(i)];
	    yDesiredWayPt = [trajectoryOpt.yWayPoint(i)];	
        desiredWayPt = [xDesiredWayPt;yDesiredWayPt];
        %% BUG RIGHT HERE!!!!
        
        %tPoint = @(t)getTPoint;
        
        minFunc = @(t)(norm(xOut((find(t>=tOut,1)),1:2)' - desiredWayPt) - 0.0001);
        [tChoice,c(i+1)] = fminbnd(minFunc,0,max(testingTSpan));        
   % toc
	end

%%%%%%%%
    %xOut(end,:) 
end

%ceq = norm((inputSys.A * pinv(inputSys.C) * desiredPos) + (inputSys.B*DPrime*basisFunc(finalTime)));

end

