% script load and analysed saved data for optimal MD control on the
% tethered mass systes using polynomial basis functions


basisOpt.order = 12;%5
tspanRange = 1.0:0.5:3.5

cols = winter(length(tspanRange));

i = 1;
for tspanMax = tspanRange
    clearvars -except tspanMax tspanRange basisOpt tspan cols i
    if(floor(tspanMax) == tspanMax)
        load(sprintf('./Data/TetheredMassFourierBasisInput/optim_tMax_%d_basisorder_%d',tspanMax,basisOpt.order),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt');
    else
       load(sprintf('./Data/TetheredMassFourierBasisInput/optim_tMax_%d_5_basisorder_%d',floor(tspanMax),basisOpt.order),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt'); 
    end
    [ result ] = testOptimalSolution( WOpt , WIni, redThreshold, tspan, mechSystem, mechOpt, basisFunc, basisOpt, cols(i,:));
    %pause;
    i= i+1;
end


figFolder = './Plots/TetheredMassFourierBasisInput/optimalFourierMulti_';
figure(1);

figTitle = 'positionWrtTime';
print('-depsc2','-r800',strcat(figFolder,figTitle));


figure(3);

figTitle = 'velocityWrtTime';
print('-depsc2','-r800',strcat(figFolder,figTitle));


close all;clear all;
basisOpt.order = 12;%5
tspanMax = 1.5;
 load(sprintf('./Data/TetheredMassFourierBasisInput/optim_tMax_%d_5_basisorder_%d',floor(tspanMax),basisOpt.order),'result','WOpt' ,'WIni', 'redThreshold', 'tspan', 'mechSystem', 'mechOpt', 'basisFunc', 'basisOpt');
[ result ] = testOptimalSolution( WOpt , WIni, redThreshold, tspan, mechSystem, mechOpt, basisFunc, basisOpt);

figFolder = './Plots/TetheredMassFourierBasisInput/optimalFourier_';
figure(1);

figTitle = 'positionWrtTime_tspan1_5';
print('-depsc2','-r800',strcat(figFolder,figTitle));

figure(2);

figTitle = 'cartesianPositione_tspan1_5';
print('-depsc2','-r800',strcat(figFolder,figTitle));


figure(3);

figTitle = 'velocityWrtTime_tspan1_5';
print('-depsc2','-r800',strcat(figFolder,figTitle));
