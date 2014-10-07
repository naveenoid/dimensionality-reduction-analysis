function robot = robotData()
%Script to set-up the simulation of the rigid kinematic chain

%% General

robot.plant               = @planar2dofArm;

%Robot initial conditions
%robot.initialCond         = [-pi/2 0 0 0 -pi/2 0 0 0 0 0]';%zeros(8,1);%[-pi/2 pi/6 0 0 -pi/2 pi/6 0 0]';%[-pi/2 0 0 0]';
robot.initialCond         = [-pi/2 0 0 0]';

%Stubs
%robot.setPointsGenerator  = @generateSetPoints;   %Generate set points
robot.trajectoryGenerator = @generateCart;        %Generate a xy traj and its derivative


% %% Simulation 
% sim.endTime             = 2;
% sim.startTime           = 0;
% sim.time                = sim.endTime-sim.startTime;

%% Controller
% 
% %Controller
% robot.ctrl.controllerFlag      = 2;%1;     %If enabled the controller is active % 1 - PID, 2 - step inputs
% robot.ctrl.controller          = @pid;
% robot.ctrl.samplingT           = 0;     %Loop period (0 for real time simulation)
% 
% robot.ctrl.controledVariables  = [1;2]; %Position (variable to which compute the error)
% robot.ctrl.inputVariables      = [3;4]; %Torques (i.e. Acceleration - indexes od the ODE)
% 
% %PID constants joint 1
% robot.ctrl.kp1                 = 200;%1250;    % [Nm/rad]
% robot.ctrl.kd1                 = 4;     % [Nm/rad/s]
% robot.ctrl.ki1                 = 150;     % []
% 
% %PID constants joint 2
% robot.ctrl.kp2                 = 150;%1500;
% robot.ctrl.kd2                 = 1;
% robot.ctrl.ki2                 = 0;

robot.tau = 0.25;

robot.muscleScale = 0.25*[7.5 0 ; 0 1.8];
robot.muscleScaleInv = inv(0.25*[7.5 0 ; 0 1.8]);

%% Physics
robot.phys.gravityAcc          = 9.81; %[m/s^2]

%Link 1
robot.phys.lengthLink1         = 0.4;    % Link length [m]
robot.phys.comLink1            = 0.2;  % Center of mass on the link frame [m]
%robot.phys.massLink1           = 0.045;    % Mass [Kg]
robot.phys.massLink1           = 0.75;%1.0;%0.15;    % Mass [Kg]
%robot.phys.massLink1           = 0.07;    % Mass [Kg]
%robot.phys.massLink1           = 0.30;    % Mass [Kg]

%Link 2
robot.phys.lengthLink2         = 0.4;0.001;    % Link length [m]
robot.phys.comLink2            = 0.2;%0.1;0.0005;  % Center of mass on the link frame [m]
%robot.phys.massLink2           = 0.045;    % Mass [Kg]
robot.phys.massLink2           = 0.5;%1.0;0.00005;    % Mass [Kg]
%robot.phys.massLink2           = 0.07;    % Mass [Kg]
%robot.phys.massLink2           = 0.30;    % Mass [Kg]


%Friction and compliance
% Minv = pinv(B);
robot.jf = 0.20;%0.25;%0.35;0.7;0.35; % joint friction;
%robot.jK = 7.5*[4.0 0; 0 2.0]; % joint stiffness
%robot.jC = 0;%-0.01*eye(2); % joint damping
%jB = 0.1*eye(2); % joint Inertia
%robot.jB = 1.0*eye(2);
%robot.jBinv = robot.jB^(-1);

%% Save configuration ina  file
%save('robotParam.mat','sim','robot');
%save('./data/robotParam.mat','robot');