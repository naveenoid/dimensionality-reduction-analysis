function [ score, redOrder, redSys, hsv, normHsv, Wc3, Wo3] = evaluateDimCost_NBT_HSV2Cost(  WHat,tspan,mechSystem, mechOpt, basisOpt, threshold, cm,redOrder  )
%EVALUATEDIMCOST_NBT_HSV2Cost  a wrapper function that computes a balanced reduction
% of a nonlinear system using the method of Hahn et al(2002).
% Params: 
% mechSystem - Function pointer to a nonlinear dynamical model of the
% system to be reduced, in the form of xDot = f(t,x,u)
% mechOpt - Structure containing parameters of the system to be reduced
%  mechOpt.inputDim - integer input dimension of the system
%  mechOpt.outputDim - integer output dimension of the system
%  mechOpt.stateDim - integer state dimension of the system
%  mechOpt.outputs - vector of integer positions of the output states
% threshold - decimal indicating the threshold below which states are
% accepted for a balanced truncation
% redOrder - an integer < mechOpt.stateDim indicating the reduced order to
% which the system is to be reduced to
%
% Naveen Kuppuswamy (2013) - naveenoid@ifi.uzh.ch

    if(size(WHat,2) == 1) 
		%Its organised as a column, presumably for optimisation, so reshape
		 WPrime = (reshape(WHat,[basisOpt.order+1,mechOpt.inputDim]))';
	else
		WPrime = WHat;
    end
    
    uss = zeros(basisOpt.order+1,1);
    xss = zeros(mechOpt.stateDim,1);
    
    mechBasisSystem = @(t,x,u)mechSystem(t,x,WPrime*u,mechOpt);
 
  %  fprintf('Obtaining Gramian time : ');
   % tic;
  % cm = max(abs(WHat));
   
    Wc3 = ctrl_gram_cov_unscaled(mechBasisSystem,[tspan(1) tspan(2) 0.1],[basisOpt.order+1 mechOpt.stateDim mechOpt.outputDim 2 100], cm, uss, xss,0);
    Wo3 = obsv_gram_cov_unscaled(mechBasisSystem,[tspan(1) tspan(2) 0.1],[basisOpt.order+1 mechOpt.stateDim mechOpt.outputDim 2 100], cm, mechOpt.outputs,uss,xss,0);
   % toc
%     
%     fprintf('Obtaining HSV time: ');
%     tic;
%     [Trans, invTrans, Wcb3, Wob3, svd_Wcb3, svd_Wob3] = bal_realization(Wc3,Wo3,mechOpt.stateDim) ;
%     hsv = ((svd_Wcb3.*svd_Wob3).^0.5)';
%     toc
 
    
  %  fprintf('Alternate HSV time');
  %  tic;
    hsvUnsorted = eig(Wc3*Wo3) .^ 0.5;
    hsv = sort(hsvUnsorted,1,'descend');%eig(Wc3*Wo3) .^ 0.5;
  %  toc;
% 
%     fprintf('Alternate HSV');
%     hsvAlt
%     
%     fprintf('Hahn HSV');
%     hsv    
        %fprintf('\n svd controllability * observability\n');

    normHsv = cumsum(hsv) ./sum(hsv);

    if(exist('redOrder','var') == 0)
        %redOrder = sum(normHsv<threshold);
        redOrder = round(interp1(normHsv,1:length(normHsv),threshold));
    end
    
   score = 100*(1-normHsv(2));% + 1-normHsv(3)) ; 
   %score = interp1(normHsv,1:length(normHsv),threshold);
   redSys = [];
  % redSys = computeReducedSys(mechSystem, mechOpt, redOrder, Trans, invTrans);
end

