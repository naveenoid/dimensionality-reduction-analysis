function [ Psi ] = gaussianBasis( t, opt )
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

    Psi = zeros(opt.order,length(t));
    %wPsi = opt.a .*(1:opt.order); %zeros(opt.order+1,length(t));
  %  Psi(1,:) = ones(size(t));
    
    for i = 1:opt.order
        Psi(i,:) =   exp( -(t-opt.b1))% cos(wPsi(i-1)*t);   %t.^(i-1);
    end
%     for i = opt.fourierOrder+2:2*opt.fourierOrder+1
%         Psi(i,:) = sin(wPsi(i - (opt.fourierOrder+1))*t);
%     end

end

