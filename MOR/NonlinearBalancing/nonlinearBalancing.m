function [ redOrder, redSys, hsv, normHsv, Wcb3, Wob3 ] = nonlinearBalancing( mechSystem, mechOpt, threshold, cm,redOrder )
%NONLINEARBALANCING a wrapper function that computes a balanced reduction
% of a nonlinear system using the method of Hahn et al(2002)
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

    uss = zeros(mechOpt.inputDim,1);
    xss = zeros(mechOpt.stateDim,1);


    Wc3 = ctrl_gram_cov_unscaled(mechSystem,[0 10 0.1],[mechOpt.inputDim mechOpt.stateDim mechOpt.outputDim 2 1000], cm , uss, xss,0);
    Wo3 = obsv_gram_cov_unscaled(mechSystem,[0 10 0.1],[mechOpt.inputDim mechOpt.stateDim mechOpt.outputDim 2 1000], cm , mechOpt.outputs,uss,xss,0);

    [Trans, invTrans, Wcb3, Wob3, svd_Wcb3, svd_Wob3] = bal_realization(Wc3,Wo3,mechOpt.stateDim) ;

        %fprintf('\n svd controllability * observability\n');

    hsv = ((svd_Wcb3.*svd_Wob3).^0.5)';

    normHsv = cumsum(hsv) ./sum(hsv);

    if(exist('redOrder','var') == 0)
        redOrder = sum(normHsv<threshold);
    end

    redSys = computeReducedSys(mechSystem, mechOpt, redOrder, Trans, invTrans);
end

