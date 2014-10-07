function theta = iKin(x,conf,opt)
%% IKIN Inverse kinematics of 2 dof planar arm
%   The computation uses the geometrical solution to the problem. It computes the joint-space 
%   positions of each joint (theta) leading to the desired position of the end-effector (x).
%
%   Inputs
%	x: Nx2 matrix containing a sequence of N points in cartesian space (x in the 1st column and y in the 2nd)
%	conf: desired robot arm configuration (to solve the redundancy problem)
%	      ['up'|'down'] for high or low elbow configuration
%	opt: options of the robot and the simulator
%	     relevant options are:
%		opt.lengthlink1: length of the 1st link
%		opt.lengthlink2: length of the 2nd link		
%
%   Output:
%	theta: Nx2 matrix containig a sequence of N joint configurations (theta1 - 1st joint - in the 1st column, 
%		theta2 - 2nd joint - in the 2nd column)
%
% Cristiano Alessandro (alessandro@ifi.uzh.ch)
%

%persistent tOld xDesOld;

a1 = opt.phys.lengthLink1;
a2 = opt.phys.lengthLink2;

c2 = ( x(:,1).^2 + x(:,2).^2 - a1^2 - a2^2 ) ./ (2*a1*a2);

if strcmp(conf,'down'); s2 = sqrt(1-c2.^2); end
if strcmp(conf,'up'); s2 = -sqrt(1-c2.^2); end
if ~(strcmp(conf,'up') || strcmp(conf,'down')) 
    disp('Wrong configuration value [up|down]');
    exit(0);
end
% TODO: handle special cases
theta2 = atan2(s2,c2);

%Singular values x==(0,0)
[i,j] = find(x==0);
diffVect = i-j;
indRow = i(diffVect==0);

s1 = ((a1+a2.*c2).*x(:,2) - a2.*s2.*x(:,1)) ./ (x(:,1).^2+x(:,2).^2);
c1 = ((a1+a2.*c2).*x(:,1) + a2.*s2.*x(:,2)) ./ (x(:,1).^2+x(:,2).^2);
theta1 = atan2(s1,c1);

%Assign theta1 as the previous value
%TODO: This is not going to work for point-to-point iKin

indFirst = find(indRow==1);
theta1(indFirst,1) = 0;
if isempty(indFirst)
    theta1(setdiff(indRow,indFirst),1) = theta1(indRow-1,1) ;
end

theta = [theta1 theta2];

end

