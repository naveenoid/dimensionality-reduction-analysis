function [ output_args ] = setupPassiveCompliantGravityKinematicChainDMPBasis( input_args )
%SETUPPASSIVECOMPLIANTGRAVITYKINEMATICCHAINDMPBASIS Summary of this function goes here

%   Detailed explanation goes here
    files{1} = './Data/DMPPaper/PassiveCompliantGravityKinematicChainDMPBasis/';
    files{2} = './Experiments/DMPPaper/PassiveCompliantGravityKinematicChainDMPInput/';
    files{3} = './MechSystems/Nonlinear/CompliantKinematicChain2LinkPassiveGravity/';
    files{4} = './MOR/NonlinearBalancing/';
    files{5} = './BasisSystem/DMPBasis/'; 

    fprintf('setupPassiveCompliantGravityKinematicChainDMPBasis executed with ');

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

