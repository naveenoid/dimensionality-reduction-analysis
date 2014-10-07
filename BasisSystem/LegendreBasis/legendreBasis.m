function [ Psi, p] = legendreBasis( t, opt )
%EXPOBASIS Summary of this function goes here
%   Detailed explanation goes here
%   PARAMS :
% t - where the expoBasis is to be evaulated
% opt - Structure which contains options
% opt.order - Order of the ExpoBasis
% 
% Naveen Kuppuswamy (2013), naveenoid@ifi.uzh.ch

% opt.order - 2n+1 basis funcs
% opt.a = n+1 size vector - coefficient of cosine terms
% opt.b = n size vector - coefficient of sine terms
% opt.w = frequency omega

p = zeros(9,9);
p(1,:) = [0              0       0           0        0          0      0             0        1];
p(2,:) = [0              0       0           0        0          0      0             1        0];
p(3,:) = [0              0       0           0        0          0      1.5000        0   -0.5000];
p(4,:) = [0              0       0           0        0       2.5000         0   -1.5000         0];
p(5,:) = [0              0       0           0    4.3750         0   -3.7500         0    0.3750];
p(6,:) = [0              0       0      7.8750         0   -8.7500         0    1.8750         0];
p(7,:) = [0              0    14.4375         0  -19.6875         0    6.5625         0   -0.3125];
p(8,:) = [0         26.8125         0  -43.3125         0   19.6875         0   -2.1875         0];
p(9,:) = [50.2734         0  -93.8438         0   54.1406         0   -9.8438         0    0.2734];


    Psi = zeros(opt.order,length(t));
    %wPsi = opt.a .*(1:opt.order); %zeros(opt.order+1,length(t));
  %  Psi(1,:) = ones(size(t));
    
    for i = 1:opt.order+1
        Psi(i,:) =   polyval(p(i,:),(2*t - opt.tmax)./opt.tmax);%exp( -(t-opt.b1))% cos(wPsi(i-1)*t);   %t.^(i-1);
    end
%     for i = opt.fourierOrder+2:2*opt.fourierOrder+1
%         Psi(i,:) = sin(wPsi(i - (opt.fourierOrder+1))*t);
%     end

end

