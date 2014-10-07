function [ output_args ] = setupTetheredMass2D_FourierBasisMDC( input_args )
%SETUPTETHEREDMASS2DFOURIERBASISINPUT setup the experiment for driving a
%tetheredmass system using Fourier Basis
%   Detailed explanation goes here

    files{1} = './Data/SynergyPaper/TetheredMass2D_FourierBasisMDC/';
    files{2} = './Experiments/SynergyPaper/TetheredMass2D_FourierBasisMDC/';
    files{3} = './MechSystems/Linear/TetheredMass2D/';
    files{4} = './MOR/LinearBalancing/';
    files{5} = './BasisSystem/FourierBasis/';
    files{6} = './BasisSystem/PolyBasis/';
    files{7} = './Controllers/Linear';   

    fprintf('setupTetheredMass2DFourierBasisInput executed with ');

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

