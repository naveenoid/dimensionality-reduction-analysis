function [opt,fName] = init2DSpringyMass(numOfInputs)

% Init Simdulator

opt.n = numOfInputs;  % no. of inputs

kx =0.1; % soft spring in center mass
ky =0.1; % soft spring in center mass

bx = 0;%0.01;
by = 0;%0.01;

%if(numOfInputs>2)
theta = pi / numOfInputs;
%else
%theta = pi

opt.A = [0 0 1 0; 0 0 0 1 ; -kx 0 -bx 0 ; 0 -ky 0 -by];
i = 0:numOfInputs-1;
opt.B = [zeros(2,numOfInputs) ; cos(i.*theta) ; sin(i.*theta)];
%[0 0 ; 0 0 ; 1 0 ; 0 1];

%opt.C = [1 0 0 0 ; 0 1 0 0]; % Only positional state variables are output
opt.C = eye(size(opt.A)); % All state variables are output
opt.D = zeros(size(opt.C,1),size(opt.B,2));

fName = './data/freeMass2D.mat';

end

