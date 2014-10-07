function [ Psi ] = fourierBasis( t,opt )
%FOURIERBASIS Evaluation of a set of Fourier Basis Functions
%   PARAMS :
% t - where the fourierBasis is to be evaulated
% opt - Structure which contains options
% opt.order - Order of the FourierBasis
% 
% Naveen Kuppuswamy (2013), naveenoid@ifi.uzh.ch

% opt.order - 2n+1 basis funcs
% opt.a = n+1 size vector - coefficient of cosine terms
% opt.b = n size vector - coefficient of sine terms
% opt.w = frequency omega

    Psi = zeros(2*opt.fourierOrder+1,length(t));
    wPsi = opt.w .*(1:opt.fourierOrder); %zeros(opt.order+1,length(t));
    Psi(1,:) = ones(size(t));
    for i = 2:opt.fourierOrder+1
        Psi(i,:) = cos(wPsi(i-1)*t);   %t.^(i-1);
    end
    for i = opt.fourierOrder+2:2*opt.fourierOrder+1
        Psi(i,:) = sin(wPsi(i - (opt.fourierOrder+1))*t);
    end
end

