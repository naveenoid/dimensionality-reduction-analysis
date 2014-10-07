function [ output ] = setupKC2lPassiveCompliant_PolyBasisMDC( input_args )
%SETUPPASSIVECOMPLIANTCHAINBASISINPUT Summary of this function goes here
%   Detailed explanation goes here

    files{1} = './Data/SynergyPaper/KC2lPassiveCompliant_PolyBasisMDC/';
    files{2} = './Experiments/SynergyPaper/KC2lPassiveCompliant_PolyBasisMDC/';
    files{3} = './MechSystems/Nonlinear/KC2lPassiveCompliant/';
    files{4} = './MOR/NonlinearBalancing/';
    files{5} = './BasisSystem/PolyBasis';  

    fprintf('setupPassiveCompliantKinematicChain executed with ');

    if nargin < 1
        for i = 1:size(files,2)
            addpath(files{i});
        end
        output = 1;
        fprintf('all paths added\n');
    else
        for i = 1:size(files,2)
            rmpath(files{i});
        end
        fprintf('all paths removed\n');
        output = 0;
    end
end