function [ tau ] = computeSteadyStateTorque( thetad, robot )
%COMPUTESTEADYSTATETORQUE Finds the steady state torque to maintain a
%desired position against gravity.
%global robot ud

opt=robot;

%%
% Dynamic model of a 2dof arm subjected to  gravity and with joint friction 
% The origin of coordinate is in the first joint: the x axis is 
% oriented to the right and the y axis upwards.
%
% t : time (optional)
%
% x : column vector contianig the state variables
%   x(1,1) = theta1, i.e. angle of the fist joint;
%   x(2,1) = theta2, i.e. angle of the second joint (wrt the first link coordinate frame);
%   x(3,1) = derivative of theta1;
%   x(4,1) = derivative of theta2.
%   All the angles according to the right-hand rule.
%
%
% Naveen Kuppuswamy (naveenoid@ifi.uzh.ch)
%

%%

    g = opt.phys.gravityAcc;

    a1 = opt.phys.lengthLink1;
    l1 = opt.phys.comLink1;
    m1 = opt.phys.massLink1;
    I1 = 1/12*m1*a1^2;

    a2 = opt.phys.lengthLink2;
    l2 = opt.phys.comLink2;
    m2 = opt.phys.massLink2;
    I2 = 1/12*m2*a2^2;

    % % Minv = pinv(B);
    % jf = robot.jf;
     jK = robot.jK; % joint stiffness
    % jC = robot.jC; %-0.01*eye(2); % joint damping
    % jB = robot.jB; % joint Inertia
    % jBinv = robot.jBinv;

    muscleScale = robot.muscleScale;

    tau = robot.tau;


    th1 = thetad(1,1);
    th2 = thetad(2,1);

    %%To handle the machine precision (pi has errors)
    c1 = cos(th1);% if c1<1e-16 && c1>-1e-16 c1=0; end
    s2 = sin(th2); %if s2<1e-16 s2=0; end
    c2 = cos(th2); %if c2<1e-16 c2=0; end
    c12 = cos(th1+th2);% if c12<1e-16 && c12>-1e-16 c12=0; end

    %G = [0; 0];
    %Joint Stiffness opposing force
    KF = (jK) * thetad;

    %fprintf('Gd\n');
    %disp(G);

    tau = muscleScale^(-1) * KF;

end

