function robot = robotData()
%Script to set-up the simulation of the rigid kinematic chain

%% General

robot.plant               = @planar2dofArm;

%Robot initial conditions
%robot.initialCond         = [-pi/2 0 0 0 -pi/2 0 0 0 0 0]';%zeros(8,1);%[-pi/2 pi/6 0 0 -pi/2 pi/6 0 0]';%[-pi/2 0 0 0]';
robot.initialCond         =  [-pi/2+pi/32 -pi/32 0 0]';%[-pi/4 pi/2 0 0]';%


%% Controller
robot.tau = 0.25;

robot.muscleScale = 0.25*[7.5 0 ; 0 1.8];
robot.muscleScaleInv = inv(0.25*[7.5 0 ; 0 1.8]);

%% Physics
robot.phys.gravityAcc          = 9.81; %[m/s^2]

%Link 1
robot.phys.lengthLink1         = 0.4;    % Link length [m]
robot.phys.comLink1            = 0.2;  % Center of mass on the link frame [m]
%robot.phys.massLink1           = 0.045;    % Mass [Kg]
robot.phys.massLink1           = 0.15;%1.0;%0.15;    % Mass [Kg]
%robot.phys.massLink1           = 0.07;    % Mass [Kg]
%robot.phys.massLink1           = 0.30;    % Mass [Kg]

%Link 2
robot.phys.lengthLink2         = 0.4;0.001;    % Link length [m]
robot.phys.comLink2            = 0.2;%0.1;0.0005;  % Center of mass on the link frame [m]
%robot.phys.massLink2           = 0.045;    % Mass [Kg]
robot.phys.massLink2           = 0.15;%1.0;0.00005;    % Mass [Kg]
%robot.phys.massLink2           = 0.07;    % Mass [Kg]
%robot.phys.massLink2           = 0.30;    % Mass [Kg]


%Friction and compliance
robot.jf = 0.15;%20;%0.25;%0.35;0.7;0.35; % joint friction;
robot.jK = 0.6;

robot.jK0 = [-pi/2+pi/32 -pi/32]';%[-pi/10 pi/5]';
%robot.jK = 7.5*[4.0 0; 0 2.0]; % joint stiffness
%robot.jC = 0;%-0.01*eye(2); % joint damping
%jB = 0.1*eye(2); % joint Inertia
%robot.jB = 1.0*eye(2);
%robot.jBinv = robot.jB^(-1);
