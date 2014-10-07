% script load and analysed saved data for optimal MD control on the
% tethered mass systes using polynomial basis functions
% 
% 
% basisOpt.order = 5;%5
% tspanRange = 1.5:0.5:3.5
% 
% cols = winter(length(1.5:0.5:3.5));
% 
% i = 1;
% for tspanMax = tspanRange
%     clearvars -except tspanMax tspanRange basisOpt tspan cols i
%     if(floor(tspanMax) == tspanMax)
%         load(sprintf('./Data/TetheredMassPolyBasisInput/optim_tMax_%d_basisorder_%d',tspanMax,basisOpt.order),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt');
%     else
%        load(sprintf('./Data/TetheredMassPolyBasisInput/optim_tMax_%d_5_basisorder_%d',floor(tspanMax),basisOpt.order),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt'); 
%     end
%     [ result ] = testOptimalSolution( WOpt , WIni, redThreshold, tspan, mechSystem, mechOpt, basisFunc, basisOpt, cols(i,:));
%     %pause;
%     i= i+1;
% end
% 
% 
% 
% figFolder = './Plots/TetheredMassBasisInput/optimalPolyMulti_';
% figure(1);
% 
% figTitle = 'positionWrtTime';
% print('-depsc2','-r800',strcat(figFolder,figTitle));
% 
% 
% figure(3);
% 
% figTitle = 'velocityWrtTime';
% print('-depsc2','-r800',strcat(figFolder,figTitle));

cols = jet(3);

%close all;%clear all;
basisOpt.order = 5;%5
tspanMax = 1.5;
load(sprintf('./Data/TetheredMassPolyBasisInput/optim_tMax_%d_5_basisorder_%d',floor(tspanMax),basisOpt.order),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt');
[ result ] = testOptimalSolution( WOpt , WIni, redThreshold, tspan, mechSystem, mechOpt, basisFunc, basisOpt,cols(1,:));

WOptStore{1} = (reshape(WOpt,[basisOpt.order+1,mechOpt.inputDim]))';

%clear all;
basisOpt.order = 12;%5
tspanMax = 1.5;
 load(sprintf('./Data/TetheredMassFourierBasisInput/optim_tMax_%d_5_basisorder_%d',floor(tspanMax),basisOpt.order),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt');
[ result ] = testOptimalSolution( WOpt , WIni, redThreshold, tspan, mechSystem, mechOpt, basisFunc, basisOpt,cols(2,:));

WOptStore{2} = (reshape(WOpt,[basisOpt.order+1,mechOpt.inputDim]))';

figFolder = './Plots/PolyFourierBasisComparison/optimalMultiBasis_';
figure(1);

figTitle = 'positionWrtTime_tspan1_5';
print('-depsc2','-r800',strcat(figFolder,figTitle));

%clear all; 

figure(2);

figTitle = 'cartesianPositione_tspan1_5';
print('-depsc2','-r800',strcat(figFolder,figTitle));


figure(3);

figTitle = 'velocityWrtTime_tspan1_5';
print('-depsc2','-r800',strcat(figFolder,figTitle));

fprintf('Poly\n');
WOptStore{1}
fprintf('Fourier\n');
WOptStore{2}