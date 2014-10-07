function ctr = pid(t,x,opt)
%%
% PID computation
%
% Cristiano Alessandro (alessandro@ifi.uzh.ch)
%

%%

ctr = zeros(size(x));
controlledVars = opt.ctrl.controledVariables;

xDes = opt.setPointsGenerator;

kp1 = opt.ctrl.kp1;
kd1 = opt.ctrl.kd1;
ki1 = opt.ctrl.ki1;

kp2 = opt.ctrl.kp2;
kd2 = opt.ctrl.kd2;
ki2 = opt.ctrl.ki2;

%e = errorPID(x(controlledVars),xDes(t),t);
e = errorPID(x(controlledVars),xDes(),t);

%ctr(controlledVars,1) = kp*e(:,:,1) + kd*e(:,:,2) + ki*e(:,:,3);
ctr(controlledVars(1),1) = kp1*e(1,:,1) + kd1*e(1,:,2) + ki1*e(1,:,3);
ctr(controlledVars(2),1) = kp2*e(2,:,1) + kd2*e(2,:,2) + ki2*e(2,:,3);

