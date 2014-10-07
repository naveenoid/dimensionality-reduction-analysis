function [ WHat, basisOpt ] = fourierBasisWeightLearning(t, thetaD, thetaDdot, fInvMech, fBasis, mechOpt,basisOpt)
%FOURIERBASISWEIGHTLEARNING Function to learn the weights for a Fourier
%basis function set for a desired trajectory. Learning is through the
%matlab fit function if only desired trajectory is given. If derivatives
%are needed the mode is not yet implemented.
% PARAMS
% t - time at which the desired trajectory is specified for tp points
% xD - Desired Position of dimension 2
%
%   Detailed explanation goes here

% number of params = 2*order of fourier basis assume basis order >= 3
WHat = zeros(mechOpt.inputDim,2*basisOpt.fourierOrder+1);

if(isempty(thetaDdot))
     % Simple learning since no xDot provided
     % Each dimension trained individually
    for i = 1:mechOpt.inputDim
         cfitObj = fit(t,thetaD(i,:)',sprintf('fourier%d',basisOpt.fourierOrder));
         basisOpt.w = cfitObj.w;
         basisOpt.order = 2*basisOpt.fourierOrder; % +1
         WHat(i,1) = cfitObj.a0;
         WHat(i,2) = cfitObj.a1;
         WHat(i,3) = cfitObj.a2;
         WHat(i,4) = cfitObj.a3;
        
         WHat(i,basisOpt.fourierOrder+2) = cfitObj.b1;
         WHat(i,basisOpt.fourierOrder+3) = cfitObj.b2;
         WHat(i,basisOpt.fourierOrder+4) = cfitObj.b3;
         
         switch(basisOpt.fourierOrder)
             case  4,
                 WHat(i,5) = cfitObj.a4;
                 WHat(i,basisOpt.fourierOrder+5) = cfitObj.b4;                 
             case 5,
                 WHat(i,5) = cfitObj.a4;
                 WHat(i,6) = cfitObj.a5;
                 WHat(i,basisOpt.fourierOrder+5) = cfitObj.b4;                 
                 WHat(i,basisOpt.fourierOrder+6) = cfitObj.b5;                 
            case 6,
                 WHat(i,5) = cfitObj.a4;
                 WHat(i,6) = cfitObj.a5;
                 WHat(i,7) = cfitObj.a5;
                 WHat(i,basisOpt.fourierOrder+5) = cfitObj.b4;                 
                 WHat(i,basisOpt.fourierOrder+6) = cfitObj.b5;
                 WHat(i,basisOpt.fourierOrder+7) = cfitObj.b6;
             case 7,     
                 WHat(i,5) = cfitObj.a4;
                 WHat(i,6) = cfitObj.a5;
                 WHat(i,7) = cfitObj.a5;
                 WHat(i,8) = cfitObj.a5;
                 WHat(i,basisOpt.fourierOrder+5) = cfitObj.b4;                 
                 WHat(i,basisOpt.fourierOrder+6) = cfitObj.b5;
                 WHat(i,basisOpt.fourierOrder+7) = cfitObj.b6;
                 WHat(i,basisOpt.fourierOrder+8) = cfitObj.b7;
             case 8,     
                 WHat(i,5) = cfitObj.a4;
                 WHat(i,6) = cfitObj.a5;
                 WHat(i,7) = cfitObj.a5;
                 WHat(i,8) = cfitObj.a5;
                 WHat(i,9) = cfitObj.a6;  
                 WHat(i,basisOpt.fourierOrder+5) = cfitObj.b4;                 
                 WHat(i,basisOpt.fourierOrder+6) = cfitObj.b5;
                 WHat(i,basisOpt.fourierOrder+7) = cfitObj.b6;
                 WHat(i,basisOpt.fourierOrder+8) = cfitObj.b7;
                 WHat(i,basisOpt.fourierOrder+9) = cfitObj.b8;
             otherwise,
                 fprintf('Fuck you naveen\n');
         end
    end
else
    if(isa(fInvMech,'function_handle'))
    % fInvMech utilised to obtain the trajectory to be "learned"
    
   
        % Construct the Wronskian for the problem and compute pseudo
        % inverse (least-squares learning)
    end
    


end

