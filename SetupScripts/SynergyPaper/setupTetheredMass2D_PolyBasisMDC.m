function [  ] = setupTetheredMass2D_PolyBasisMDC( opt )
%SETUPRIGIDKINEMATICCHAINBASISINPUT Summary of this function goes here
%   Detailed explanation goes here

    files{1} = './Data/TetheredMassPolyBasisInput/';
    files{2} = './Experiments/TetheredMassPolyBasisInput/';
    files{3} = './MechSystems/Linear/TetheredMass2D/';
    files{4} = './MOR/LinearBalancing/';
    files{5} = './BasisSystem/PolyBasis/';   
    files{6} = './Controllers/Linear';   

    fprintf('setupTetheredMass2DBasisInput executed with ');

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

