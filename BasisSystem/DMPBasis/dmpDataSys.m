function [ uDesInt ] = dmpDataSys( t,opt )
% DMPDATASYS Allows usage of the DMP Basis set in a integrator. It
% interpolates within a provided dataset of DMPs of fixed basis size.
% Dataset is expected to be of dimension (b+2,nT) for a system with b basis
% and nT time steps

 % INTERPOLATE TO FIND u(t)   
 uDesInt = interp1(opt.tDes,opt.psiStar',t)';  
 

end

