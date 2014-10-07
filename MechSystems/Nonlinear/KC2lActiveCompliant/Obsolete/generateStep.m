function setPoints = generateStep(init,step,t)
%GENERATESTEP Summary of this function goes here
%   Detailed explanation goes here

%setPointsJ1  = [(init(1)+step)*pi/180*ones(1,length(t)) ; zeros(1,length(t))];
setPointsJ1  = [(init(1)+step)*ones(1,length(t)) ; zeros(1,length(t))];

setPointsJ2  = [init(1)*ones(1,length(t)) ; (init(2)+step)*pi/180*ones(1,length(t))];

setPointsJ12  = [(init(1)+step)*pi/180*ones(1,length(t)) ; (init(2)+step)*pi/180*ones(1,length(t))];

setPoints = setPointsJ1;

end

