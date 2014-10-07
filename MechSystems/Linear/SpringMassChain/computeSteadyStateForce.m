function [ fOut ] = computeSteadyStateForce( xDes, opt )
%COMPUTESTEADYSTATEFORCE Summary of this function goes here
%   Detailed explanation goes here

    fOut = xDes*ones(size(opt.B,2),1);

end

