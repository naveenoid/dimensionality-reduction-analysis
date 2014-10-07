% function dydt = sysLTI(t,y,opt)
% function to simulate an LTI System
% Naveen Kuppuswamy (naveenoid@ifi.uzh.ch)
% Last Modified 1/07/2011

function dxdt = sysLTIForced(t,x,u,opt)
%controlFunc = opt.controlFunc;
%dxdt = opt.A * x + opt.B*uDes;
    dxdt = opt.A * x + opt.B*u;
end