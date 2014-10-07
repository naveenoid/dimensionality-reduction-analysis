function dxdt = planar2dofArm(t,x,ud,robot)

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

% Minv = pinv(B);
jf = robot.jf;
jK = robot.jK; % joint stiffness
jK0 = robot.jK0;
% jC = robot.jC; %-0.01*eye(2); % joint damping
% jB = robot.jB; % joint Inertia
% jBinv = robot.jBinv;

muscleScale = robot.muscleScale;

tau = robot.tau;


th1 = x(1,1);
th2 = x(2,1);
th1d = x(3,1);
th2d = x(4,1);

%%To handle the machine precision (pi has errors)
c1 = cos(th1); if c1<1e-16 && c1>-1e-16 c1=0; end
s2 = sin(th2); %if s2<1e-16 s2=0; end
c2 = cos(th2); %if c2<1e-16 c2=0; end
c12 = cos(th1+th2); if c12<1e-16 && c12>-1e-16 c12=0; end

% % Inertia matrix (B)
% b11 = m1*l1^2 + I1 + m2*a1^2 + m2*l2^2 + 2*m2*a1*l2*c2 + I2;
% b12 = m2*l2^2 + m2*a1*l2*c2 + I2;
% b21 = m2*l2^2 + m2*a1*l2*c2 + I2;
% b22 = m2*l2^2 +I2;
% B = [ b11 b12; b21 b22];

% Coriolis matrix (C)
c11m = -2*a1*l2*m2*s2*th2d;
c12m = -a1*l2*m2*s2*th2d;
c21m = a1*l2*m2*s2*th1d;
c22m = 0;
C = [c11m c12m; c21m c22m];

% Gravity matrix (G)
%g1 = (m1*l1 + m2*a1)*g*c1 + m2*g*l2*c12;
%g2 = m2*g*l2*c12;
%G = [g1; g2];
%G = [0; 0];
% Minv =
%                                                            
% [                (m2*l2^2 + I2)                           -(m2*l2^2 + a1*c2*m2*l2 + I2)]
% [ -(m2*l2^2 + a1*c2*m2*l2 + I2) (m2*a1^2 + 2*c2*m2*a1*l2 + m1*l1^2 + m2*l2^2 + I1 + I2)]
%  
% (- a1^2*c2^2*l2^2*m2^2 + a1^2*l2^2*m2^2 + I2*a1^2*m2 + m1*l1^2*l2^2*m2 + I2*m1*l1^2 + I1*l2^2*m2 + I1*I2)
% (- a1^2*c2^2*l2^2*m2^2 + a1^2*l2^2*m2^2 + I2*a1^2*m2 + m1*l1^2*l2^2*m2 + I2*m1*l1^2 + I1*l2^2*m2 + I1*I2)
% (- a1^2*c2^2*l2^2*m2^2 + a1^2*l2^2*m2^2 + I2*a1^2*m2 + m1*l1^2*l2^2*m2 + I2*m1*l1^2 + I1*l2^2*m2 + I1*I2)
% (- a1^2*c2^2*l2^2*m2^2 + a1^2*l2^2*m2^2 + I2*a1^2*m2 + m1*l1^2*l2^2*m2 + I2*m1*l1^2 + I1*l2^2*m2 + I1*I2)

MinvDen = (I2+l2^2*m2)*(I2+I1+2*a1*l2*m2*c2+(l2^2+a1^2)*m2+l1^2*m1)+ ...
    (-I2-a1*l2*m2*c2-l2^2*m2)*(I2+a1*l2*m2*c2+l2^2*m2);
MinvNom = [I2+l2^2*m2                  -I2-a1*l2*m2*c2-l2^2*m2 ;
        -I2-a1*l2*m2*c2-l2^2*m2     I2+I1+2*a1*l2*m2*c2+(l2^2+a1^2)*m2+l1^2*m1];
Minv = 1/MinvDen .* MinvNom;

if isa(ud, 'function_handle')
    uInp = ud(t);
else
    uInp = ud;
end

tauIn = muscleScale * uInp;

dxdt(1:4,1) = zeros(4,1);

dxdt(1:2,1) = x(3:4,1);
dxdt(3:4,1) = -Minv*(C*x(3:4,1)+ jf*x(3:4,1) +jK*(x(1:2,1) - jK0)- tauIn );% + jK*(x(1:2,1) - x(5:6,1) +jC*(x(3:4,1) - x(7:8,1))));
% dxdt(5:6,1) = x(7:8,1);
% dxdt(7:8,1) = jBinv*(-jK*(x(5:6,1) - x(1:2,1))-jC*(x(7:8,1) - x(3:4,1))); 

%%forced step control
% dxdtControl = muscleScale*uInp;%[0.01;-0.01];%[0.04;0.01];
% dxdt = dxdt +  [0;0;0;0;0;0;jBinv*dxdtControl]; % forces added to 2nd order of actuator angle

end


