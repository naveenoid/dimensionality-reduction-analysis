function [ x] = fitSigmoid( t,xStart,xEnd )
%FITSIGMOID Summary of this function goes here
%   Detailed explanation goes here

if(size(t,1)>size(t,2))
    t=t';
end

x(1,:) = repmat(xStart(1),1,length(t)) + 0.5.*diag(xEnd(1)-xStart(1))*(1 + tanh(sign(xStart(1)-xEnd(1))*(t - 0.5*max(t))));
x(2,:) = repmat(xStart(2),1,length(t)) + 0.5.*diag(xEnd(2)-xStart(2))*(1 + tanh(-sign(xStart(2)-xEnd(2))*(t - 0.5*max(t))));
end

