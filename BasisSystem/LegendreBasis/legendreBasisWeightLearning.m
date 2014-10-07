function [  WHat, basisOpt ] = legendreBasisWeightLearning( t, thetaD, thetaDdot, fInvMech, fBasis, mechOpt,basisOpt )
%EXPOBASISWEIGHTLEARNING Summary of this function goes here
%   Detailed explanation goes here

WHat = zeros(mechOpt.inputDim,basisOpt.order+1);

%legendreOpt.tmax = max(t);
if(isempty(thetaDdot))
     % Simple learning since no xDot provided
     % Each dimension trained individually
    for i = 1:mechOpt.inputDim        
            
         temp = legendrefit(thetaD(i,:), basisOpt.order); %polyfit(t,thetaD(i,:)',basisOpt.order);
         WHat(i,:) = temp';
         % reversing since in the matlab poly usage and usage in this 
         % library are flipped back to front (i.e. index 1 is the scaling 
         % of t^0 for t)
    end
% else
%     if(isa(fInvMech,'function_handle'))
%     % fInvMech utilised to obtain the trajectory to be "learned"
%         [uFunc] = fInvMech(t);
%         for i = 1:mechOpt.inputDim
%             temp = polyfit(t,uFunc{i}(t),basisOpt.order);
%             WHat(i,:) = temp';%temp(end:-1:1) .* (basisOpt.tmax.^(0:basisOpt.order)); 
%             % reversing since in the matlab poly usage and usage in this 
%             % library are flipped back to front (i.e. index 1 is the scaling 
%             % of t^0 for t)
%         end
%         
%         
%     end
    


end

