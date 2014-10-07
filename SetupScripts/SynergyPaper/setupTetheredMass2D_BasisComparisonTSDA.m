function [ output_args ] = setupTetheredMass2D_BasisComparisonTSDA( input_args )
%SETUPPOLYFOURIERBASISCOMPARISON Script to setup the polyBasis-Fourier
%Basis Comparison on the Tethered Mass System.
%   For adding the necessary paths no input arguments must be provided. If any
%   input argument is used, the paths are removed.


  %  files{1} = './Data/PolyFourierBasisComparisonTetheredMass/';
    files{1} = './Experiments/SynergyPaper/TetheredMass2D_BasisComparisonTSDA/';
    files{2} = './MechSystems/Linear/TetheredMass2D/';
    files{3} = './MOR/LinearBalancing/';
    %files{4} = './BasisSystem/PolyBasis/';   
    files{4} = './BasisSystem/FourierBasis/';   
    files{5} = './BasisSystem/LegendreBasis/';   
    files{6} = './Controllers/Linear';   

    fprintf('setupPolyFourierBasisComparisonTetheredMass : ');

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

