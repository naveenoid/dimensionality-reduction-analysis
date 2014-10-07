function [ output_args ] = setupKC2lPassiveCompliant_MDCAnalysis( input_args )
%SETUPKC2LPASSIVECOMPLIANT_MDCANALYSIS Summary of this function goes here
%   Detailed explanation goes here
   % files{1} = './Data/TetheredMassPolyBasisInput/';
    files{1} = './Experiments/SynergyPaper/KC2lPassiveCompliant_MDCAnalysis/';
    files{2} = './MechSystems/Nonlinear/KC2lPassiveCompliant/';
    files{3} = './MOR/NonlinearBalancing/';
    files{4} = './BasisSystem/LegendreBasis/';   
    files{5} = './BasisSystem/FourierBasis/';%'./Controllers/Linear';      
    files{6} = './Controllers/Linear';  

    fprintf('setupTetheredMass2DOptimAnalysis executed with ');

     if nargin < 1
        for i = 1:size(files,2)
            addpath(files{i});
        end
        fprintf('all paths added\n');
    else
        for i = 1:size(files,2)
            rmpath(files{i});
        end
        fprintf('all paths removed\n');
    end


end

