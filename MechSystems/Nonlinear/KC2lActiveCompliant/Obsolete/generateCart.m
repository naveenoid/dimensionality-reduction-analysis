function traj = generateCart(t,T)
%% Stub to generate a trajectory in cartesian space
%
% Cristiano Alessandro (alessandro@ifi.uzh.ch)
%

persistent tOld xDesOld;

%Compute the trajectory
f = 1/T;
r = 1.5;
x = r*cos(2*pi*f*t);
y = r*sin(2*pi*f*t);

xDes = [x;y];

%Compute the derivative of the trajectory
if isempty(xDesOld)
    xDesdt = zeros(2,1);
else
    %if t==zeros(size(t,1))
    if t==tOld
        xDesdt = zeros(2,1);
    else
        dt = t - tOld;
        xDesdt = (xDes-xDesOld)./dt;
    end    
end
tOld = t;
xDesOld = xDes;

traj = [xDes;xDesdt];

end


