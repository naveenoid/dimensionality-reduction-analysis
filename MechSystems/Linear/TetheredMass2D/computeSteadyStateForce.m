function [ F ] = computeSteadyStateForce( xDes , opt )
%COMPUTESTEADYSTATEFORCE Calculate the steady state force needed for a
%tethered mass to reach a point in space
% Param :
%  xDes : desired position in x, and y.
%  mechOpt : struct containing parameters of tethered mass
    Fx = opt.kx*xDes(1);
    Fy = opt.ky*xDes(2);
    
    F = [Fx;Fy];

end