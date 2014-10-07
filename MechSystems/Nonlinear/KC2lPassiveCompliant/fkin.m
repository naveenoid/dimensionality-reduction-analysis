function xyzSol = fkin( theta1, theta2, opt )
%%FKIN Foward kinematics
% Computes the cartesian positions of the two links' end-points of the robot from the correspoinding joint space positions. 
%
%   Inputs:
%	theta1: Nx1 vector containing the sequnce of N 1st joint's positions (in joint space)
%	theta2: Nx1 vector containing the sequnce of N 2nd joint's positions (in joint space)
%	opt: robot and simulation options
%		relevant options are
%		opt.lengthlink1: length of the 1st link
%		opt.lengthlink2: length of the 2nd link
%
%   Output:
%	xyzSol: Nx4 matrix containing the N cartesian positions of the 1st and 2nd link' end points
%		xyzSol(i,1): x position of the 1st link's end point (at the i-st time step)
%		xyzSol(i,2): y position of the 1st link's end point (at the i-st time step)
%		xyzSol(i,3): x position of the 2st link's end point (at the i-st time step)
%		xyzSol(i,4): y position of the 2st link's end point (at the i-st time step)
%
%   Cristiano Alessandro (alessandro@ifi.uzh.ch)
%

x1 = opt.phys.lengthLink1.*cos(theta1);
y1 = opt.phys.lengthLink1.*sin(theta1);

x2 = opt.phys.lengthLink1.*cos(theta1) + ...
    opt.phys.lengthLink2.*cos(theta1+theta2);
y2 = opt.phys.lengthLink1.*sin(theta1) + ...
    opt.phys.lengthLink2.*sin(theta1+theta2);

xyzSol = [x1, y1, x2, y2];

end

