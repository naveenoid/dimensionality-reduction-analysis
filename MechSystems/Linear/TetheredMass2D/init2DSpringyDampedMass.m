function [opt,fName] = init2DSpringyDampedMass(numOfInputs)

% Init Simdulator

opt.n = numOfInputs;  % no. of inputs

kx =6;%6;%6;%6;%2;%3;%6;%0.1 % soft spring in center mass
ky =6;%6;%6;%6;%2;%3;%6;%0.1 % soft spring in center mass

% TSDA = 6,6
% MDC = 2,2

bx = 4.0;%0.04;%2;%0.4%0.4;0.06;%2;
by = 4.0;%0.04;%2;%0.4;0.06;%2;

% TSDA = 4,4
% MDC = 0.4,0.4

%if(numOfInputs>2)
theta = pi / numOfInputs;
%else
%theta = pi

opt.kx = kx;
opt.ky = ky;
opt.bx = bx;
opt.by = by;

opt.A = [0 0 1 0; 0 0 0 1 ; -kx 0 -bx 0 ; 0 -ky 0 -by];
i = 0:numOfInputs-1;
opt.B = [zeros(2,numOfInputs) ; cos(i.*theta) ; sin(i.*theta)];
%[0 0 ; 0 0 ; 1 0 ; 0 1];

opt.C = [1 0 0 0 ; 0 1 0 0]; % Only positional state variables are output
%opt.C = eye(size(opt.A)); % All state variables are output
opt.D = zeros(size(opt.C,1),size(opt.B,2));

fName = './data/freeMass2D.mat';

end

