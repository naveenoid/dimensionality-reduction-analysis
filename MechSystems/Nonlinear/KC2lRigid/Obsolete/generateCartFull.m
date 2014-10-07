function traj = generateCartFull(t,type,robot,train)
%% Stub to generate a trajectory in 2D cartesian space in off-line mode
%
%    Input
%	t: Nx1 vector containing the N time stamps in which to compute the points of the trajectory
%	opt: options of robot and simulator
%		relevant options are
%		opt.lengthlink1: length of the 1st link
%		opt.lengthlink2: length of the 2nd link	
%
%   Output
%	traj: Nx4 matrix containing N end-points and corresponding velocities of the desired trajectory
%		traj(i,1): i-th x position of the end-effector
%		traj(i,2): i-th y position of the end-effector 
%		traj(i,3): derivative of the i-th x position of the end-effector 
%		traj(i,4): derivative of the i-th y position of the end-effector 
%
%   Cristiano Alessandro (alessandro@ifi.uzh.ch)
%

%Compute the trajectory
%f = 1/max(t);
%r = 1.5;
%x = r*cos(2*pi*f*t);
%y = r*sin(2*pi*f*t);

switch type
    case 'eight'
        a = (robot.phys.lengthLink1+robot.phys.lengthLink2)/3;
        x = a * sin(t);
        y = a * sin(t).*cos(t);
        
    case 'circle'
        omg = 0.7;
        r = (robot.phys.lengthLink1+robot.phys.lengthLink2)/3;
        x = r*cos(omg*t);
        y = r*sin(omg*t);
        
    case 'ellipse'
        omg = 0.7;
        a = (robot.phys.lengthLink1+robot.phys.lengthLink2)/3;
        b = (robot.phys.lengthLink1+robot.phys.lengthLink2)/2;
        x = a*cos(omg*t);
        y = b*sin(omg*t);
        
    case 'verticalLine'
        a = 2/3;
        x = a * (robot.phys.lengthLink1+robot.phys.lengthLink2);
        
        y0 = -0.95 * (robot.phys.lengthLink1+robot.phys.lengthLink2) * sqrt(1-a^2);
        %x0 = -0.3;
        yf = -y0/2;
        
        a0 = y0;
        a1 = 0;
        a2 = 3*abs(yf-y0)/train.endTime^2;
        a3 = -2*abs(yf-y0)/train.endTime^3;
        
        %x = 2/3 * (robot.phys.lengthLink1+robot.phys.lengthLink2);
        x = x.*ones(size(t,1),1);
        y = a3.*t.^3 + a2.*t.^2 + a1.*t + a0;      
        
    case 'horizontalLine'
        a = 2/3;
        y = a * (robot.phys.lengthLink1+robot.phys.lengthLink2);
        
        x0 = -0.95 * (robot.phys.lengthLink1+robot.phys.lengthLink2) * sqrt(1-a^2);
        %x0 = -0.3;
        xf = -x0/2;
        
        a0 = x0;
        a1 = 0;
        a2 = 3*abs(xf-x0)/train.endTime^2;
        a3 = -2*abs(xf-x0)/train.endTime^3;
        
        x = a3.*t.^3 + a2.*t.^2 + a1.*t + a0; 
        y = y.*ones(size(t,1),1);
       
end

xDes(:,1:2) = [x y];

dt = repmat(diff(t),1,2);

xDesdt = [  0   0;
             diff(xDes(:,1:2))./dt];
        
traj = [xDes xDesdt];

end
