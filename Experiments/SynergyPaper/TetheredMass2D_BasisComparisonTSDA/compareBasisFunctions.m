%polyBasis setup;
basisFunc{1} = @polyBasis;
basisLearning{1} = @polyBasisWeightLearning;
basisOpt{1}.order = 5;%12; 
inputDim{1} = basisOpt{1}.order+1;


%polyBasis setup;
basisFunc{2} = @fourierBasis;
basisLearning{2} = @fourierBasisWeightLearning;
basisOpt{2}.fourierOrder = 5;%12;  %a number between 3 and 8
%basisOpt{2}.order = 7;%12; 