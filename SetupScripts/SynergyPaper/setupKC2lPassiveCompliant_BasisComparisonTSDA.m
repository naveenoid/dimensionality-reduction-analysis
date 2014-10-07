function [ output ] = setupKC2lPassiveCompliant_BasisComparisonTSDA( input_args )
%SETUPKC2LPASSIVECOMPLIANT_BASISCOMPARISONTSDA Script to setup the TSDA
%basis comparison experiment
%   Detailed explanation goes here

    files{1} = './Data/SynergyPaper/KC2lPassiveCompliant_BasisComparisonTSDA/';
    files{2} = './Experiments/SynergyPaper/KC2lPassiveCompliant_BasisComparisonTSDA/';
    files{3} = './MechSystems/Nonlinear/KC2lPassiveCompliant/';
    files{4} = './MOR/NonlinearBalancing/';
    files{5} = './BasisSystem/LegendreBasis/';   
    files{6} = './BasisSystem/FourierBasis/';   

    fprintf('setupKC2lPassiveCompliant_BasisComparisonTSDA executed with ');

     if nargin < 1
        for i = 1:size(files,2)
            addpath(files{i});
        end
        fprintf('all paths added\n');
        output = 1;
    else
        for i = 1:size(files,2)
            rmpath(files{i});
        end
        fprintf('all paths removed\n');
        output = 0;
    end

end

