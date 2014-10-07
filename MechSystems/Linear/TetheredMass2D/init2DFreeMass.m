function [opt,fName] = init2DFreeMass(numOfInputs)

% Init Simdulator

opt.n = 2;  % no. of inputs

theta = pi / numOfInputs;

opt.A = [0 0 1 0; 0 0 0 1 ; 0 0 0 0 ; 0 0 0 0];
i = 0:numOfInputs-1;
opt.B = [zeros(2,numOfInputs) ; cos(i.*theta) ; sin(i.*theta)];
opt.C = [1 0 0 0 ; 0 1 0 0];
opt.D = zeros(size(opt.C,1),size(opt.B,2));

fName = './data/freeMass2D.mat';

end

