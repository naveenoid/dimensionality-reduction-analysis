function [ opt] = init1DChain(chainLength)
%init1DChain( dampingRatio, chainLength )
% Copyright (C) 2011 Naveen Kuppuswamy (naveenoid@ifi.uzh.ch)
% This file is part of MORLTI
% usage: [opt, fName] = init1DChain (dampingRatio, chainLength )
%
% Build the state matrix for a 1D chain of springs and masses. opt is a struct that contains the system, input, output and feedthrough matrices (A,B,C, and D respectively).
% Arguments : 
% dampingRatio : Ratio between linear spring and damping coefficients (applied uniformly throughout the chain)
% chainLength : Number of masses the chain is composed of.
%	
% Author : Naveen Kuppuswamy (naveenoid@ifi.uzh.ch), 2011

% initialisations
dampingRatio = 0.250;%0.25;
inputScale = 1.0;%1.0e3;
k = 1.0*ones(chainLength,1); 
massDistribution = 1.0*ones(1,chainLength);

l0 = zeros(chainLength,1);
d = dampingRatio*ones(chainLength,1);
%opt.k = k;



%    dampingRatio <
ID = eye(chainLength);
DAMP = diag(-d./massDistribution') ;%* repmat(1./massDistribution,chainLength,1);
ZER = zeros(chainLength);

diagEle  = k(2:end) + k(1:end-1);
diagEle(end+1) = k(end);

%opt.k = diag(-diagEle,0) + diag(k(2:end),1) + diag(k(2:end),-1) ;
%opt.kInv = (opt.k)^-1;

STIFF = (diag(-diagEle,0) + diag(k(2:end),1) + diag(k(2:end),-1)) .* repmat(1./massDistribution,chainLength,1);

opt.A = [ZER ID; STIFF DAMP];
opt.B = [zeros(chainLength) ; diag(inputScale./massDistribution)] ;%Force actuators on each mass
%opt.B = [zeros(chainLength,1);1;zeros(chainLength-1,1)];
%opt.B = [zeros(chainLength,1);1;zeros(chainLength-1,1)];

%opt.Binp = [1;zeros(chainLength-1,1)];

%opt.C = [zeros(1,chainLength-1) 1 zeros(1,chainLength);zeros(1,chainLength) zeros(1,chainLength-1) 1];
opt.C = [zeros(1,chainLength-1) 1 zeros(1,chainLength)];
%opt.C = [eye(chainLength) zeros(chainLength,chainLength)];%zeros(1,chainLength) zeros(1,chainLength-1) 1];
%opt.C = [eye(chainLength) zeros(chainLength)];%eye(size(opt.A,1));%zeros(1,chainLength) zeros(1,chainLength-1) 1];
%opt.C = eye(size(opt.A,1));
opt.D = zeros(size(opt.C,1),size(opt.B,2));

% Output is the position and velocity of the end mass

opt.chainLength = chainLength; % number of pointmasses in the chain
%opt.restLength = l0; % vector of the rest lengths

opt.STIFF = STIFF;
opt.DAMP = DAMP;
opt.invMassD = diag(1./massDistribution');
opt.massD = diag(massDistribution');
opt.inputScale = inputScale;
%fName = './data/1DChain.mat';

end

