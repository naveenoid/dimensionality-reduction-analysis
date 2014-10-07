function [ redSys, redSysOpt ] = computeReducedSys(mechSystem, mechOpt, redOrder, Trans, invTrans)
%COMPUTEREDUCEDSYS accepts a function pointer to a nonlinear dynamical
%system to be reduced by balancing truncation and produces a truncated
%system as output in the form of a standard nonlinear dynamical system (
%xdot = f(t,x,u)
% PARAMS :
% mechSystem - pointer to a full dimensional dynamical system in the form
% xdot = f(t,x,u)
% mechOpt - structure containing parameters for the dynamical system

    redSys = @(t,y,u)reducedSystem(mechSystem,t,y,u,Trans, invTrans,mechOpt.stateDim, redOrder);
    redSysOpt.stateDim = redOrder;
    redSysOpt.inputDim = mechOpt.inputDim;
    redSysOpt.outputDim = mechOpt.outputDim;
    redSysOpt.outputs = mechOpt.outputs;
end

