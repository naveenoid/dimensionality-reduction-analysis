tspan = [1,4.0];
%load(sprintf('./Data/PolyFourierBasisComparison/basisComparison_tspan_%d',round(10*tspan(2))),'result', 'resultMech', 'trajectoryName', 'xD','redThreshold', 'numBasis', 'numTestTraj', 'cols','mechOpt','costFuncStore','basisFunc', 'basisLearning', 'basisOpt', 'inputDim','result', 'resultMech', 'trajectoryName', 'xD','redThreshold', 'numBasis', 'numTestTraj', 'cols','mechOpt','costFuncStore','hsvCostFuncStore','basisFunc', 'basisLearning', 'basisOpt', 'inputDim');
%plotCartesianTrajectories( result, resultMech, trajectoryName, xD, redThreshold, numBasis, numTestTraj, cols,mechOpt,costFuncStore,hsvCostFuncStore,basisFunc, basisLearning, basisOpt, inputDim )
for i = 1:5
    figure(i);
end

load(sprintf('./Data/PolyFourierBasisComparison/basisComparisonKChain_tspan_%d',round(10*tspan(2))),'result', 'resultMech', 'trajectoryName', 'xD','redThreshold', 'numBasis', 'numTestTraj', 'cols','mechOpt','costFuncHSV2Store','costFuncRedOrderStore','basisFunc', 'basisLearning', 'basisOpt', 'inputDim');
plotCartesianTrajectories( result, resultMech, trajectoryName, xD,redThreshold, numBasis, numTestTraj, cols,mechOpt,costFuncHSV2Store,costFuncRedOrderStore,basisFunc, basisLearning, basisOpt, inputDim );