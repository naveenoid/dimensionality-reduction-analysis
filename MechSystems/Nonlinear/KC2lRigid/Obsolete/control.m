function dxdt = control(t,x,Minv,opt)
%%CONTROL Function handling the control of the plant. 
%This function produces the ODE (uncontrolled plant + controller) at timestep t.
%
%   Inputs:
%	t: current timestamp (1x1 vector)
%	x: current state of the system 
%		x(1,1): theta 1 
%		x(2,1): theta 2
%		x(3,1): derivative of theta1
%		x(4,1): derivative of theta2
%	Minv: inverse of the Mass matrix
%	opt: robot and simulation options
%		relevant options are:
%		opt.controller: defines the controller to be used
%		opt.samplingT: defines the controller sampling time (0 = real-time)
%
%   Output:
%	dxdt: vector containing the ODEs conputation at timestamp t
%		dxdt(1,1): derivative of theta 1
%		dxdt(2,1): derivative of theta 2
%		dxdt(3,1): second derivative of theta 1
%		dxdt(4,1): second derivative of theta 2
%
% Cristiano Alessandro (alessandro@ifi.uzh.ch)
%

persistent T t_prev ind Fcontrol;

% Controller 
controller = opt.ctrl.controller; 
% Controller sampling period
samplingT = opt.ctrl.samplingT;

if isempty(T)
    %disp('Initializing controlled plant')
    fprintf('       Initializing controlled plant\n');
    T = 0;
    t_prev = t;
    ind = 1;
%     Fcontrol(:,ind) = controller(t,x,opt);
    Fcontrol = controller(t,x,opt);
else
%    ind = ind + 1;
    ind = 0;
end

% Accumulates time passed since last update of the controller
T = T + (t-t_prev);

if T >= samplingT && ind~=1
    T = 0;
    %Fcontrol(:,ind) = controller(t,x,opt);
    Fcontrol = controller(t,x,opt);
end

% tmp = Fcontrol(1:2,ind);
% Fcontrol(1:2,ind) = Fcontrol(3:4,ind);
% Fcontrol(3:4,ind) = tmp;
% 
% dxdt = Minv*Fcontrol(:,ind);

tmp = Fcontrol(1:2);
Fcontrol(1:2) = Fcontrol(3:4);
Fcontrol(3:4) = tmp;

dxdt = Minv*Fcontrol;

t_prev=t;
