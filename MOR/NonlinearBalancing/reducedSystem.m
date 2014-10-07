function [ dxdt ] = reducedSystem(fullSystem,t,y, ud,Trans, invTrans, n,red_n)
%REDUCEDSYSTEM Summary of this function goes here
%   Detailed explanation goes here

x = invTrans*y;

dxdt = fullSystem(t,x,ud);

dxdt(1:red_n,1) 	= Trans(1:red_n,:)*dxdt;
dxdt(red_n+1:n,1) =  zeros(n - red_n,1);

end

