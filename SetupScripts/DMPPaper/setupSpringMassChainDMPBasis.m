function [ output_args ] = setupSpringMassChainDMPBasis( input_args )
%SETUPSPRINGMASSCHAINDMPBASIS Summary of this function goes here
%   Detailed explanation goes here

%   Detailed explanation goes here
    files{1} = './Data/DMPPaper/SpringMassChainDMPBasisInput/';
    files{2} = './Experiments/DMPPaper/SpringMassChainDMPBasisInput/';
    files{3} = './MechSystems/Linear/SpringMassChain/';
    files{4} = './MOR/LinearBalancing/';
    files{5} = './BasisSystem/DMPBasis/'; 

    fprintf('setupSpringMassChainDMPBasis executed with ');

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

