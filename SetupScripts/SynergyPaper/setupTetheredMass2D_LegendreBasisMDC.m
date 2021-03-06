function [  ] = setupTetheredMass2D_LegendreBasisMDC( opt )
%SETUPRIGIDKINEMATICCHAINBASISINPUT Summary of this function goes here
%   Detailed explanation goes here

    files{1} = './Data/SynergyPaper/TetheredMass2D_LegendreBasisMDC/';
    files{2} = './Experiments/SynergyPaper/TetheredMass2D_LegendreBasisMDC/';
    files{3} = './MechSystems/Linear/TetheredMass2D/';
    files{4} = './MOR/LinearBalancing/';
    files{5} = './BasisSystem/LegendreBasis/';   
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

