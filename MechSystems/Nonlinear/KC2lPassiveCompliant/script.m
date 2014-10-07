%% Script
%clear all;
robotData;

%% Inital conditions
xi = [0 0 0 0];

%% Time interval
acc = 1e-4;
maxTime = sim.endTime;
minTime = sim.startTime;

%timespan = linspace(minTime,maxTime,round(opt.time/acc));
%timespan = linspace(minTime,maxTime,3);
timespan = [minTime maxTime];

%% setup integration
uncontrolledPlant = opt.plant;
sys =@(t,x)uncontrolledPlant(t,x,robot);

options = odeset('InitialStep',1e-10, 'MaxStep', 1e-4);

%[t sol]=ode45(sys,timespan,xi);
%[t sol]=ode15s(sys,timespan,xi);
[t sol]=ode15s(sys,timespan,xi,options);
%[t sol]=ode45(sys,timespan,xi,options);

