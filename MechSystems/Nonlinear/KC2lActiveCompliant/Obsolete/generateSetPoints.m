function setPoints = generateSetPoints(t,train)
%% Stub to generate setPoints to each joint
%
% Cristiano Alessandro (alessandro@ifi.uzh.ch)
%

%fConst  = [-pi/2*ones(1,length(t)) ; pi/2*ones(1,length(t))];
%fConst  = [-pi/2*ones(1,length(t)) ; 0*ones(1,length(t))];
%fConst  = [zeros(1,length(t)) ; zeros(1,length(t))];
%fConst  = [3*pi/180*ones(1,length(t)) ; zeros(1,length(t))];
%fConst  = [zeros(1,length(t)) ; pi/2*ones(1,length(t))];
%fOsc    = [-pi/2*ones(1,length(t)) ; cos(t)];

%th0 = iKin(opt.robot.initialCond(1:2)','down',opt.robot);
th0 = [0 0];
thf = [pi/2 5/6*pi];

a0 = th0;
a1 = [0 0];
a2 = 3*abs(thf-th0)/train.endTime^2;
a3 = -2*abs(thf-th0)/train.endTime^3;

th = t.^3*a3 + t.^2*a2 + t*a1 + ones(size(t,1),1)*a0;

fTraj = th;

setPoints = fTraj;

end

