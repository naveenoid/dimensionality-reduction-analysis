

function [ output_args ] = pathadder( experimentName )
%PATHADDER function to add path of the setupscript folder of the project
% turn on parallel processing by setting the variable parallel to a nonzero
% value

root = 'SetupScripts';
if nargin < 1
    addpath('./SetupScripts');
else
    experiment  = lower(experimentName);
    if(strcmp(experiment,'dmp')==1)
        addpath('./SetupScripts/DMPPaper/');
        root = 'DMPPaper';
    elseif(strcmp(experiment,'synergy'))
        addpath('./SetupScripts/SynergyPaper/');
        root = 'SynergyPaper';
    elseif(strcmp(experiment,'dimchange'))
        addpath('./SetupScripts/DimChangePaper/');
        root = 'DimChangePaper';
    else
        addpath('./SetupScripts');
    end
end
        
%printf('All paths added\n');
fprintf('Path of %s added\n',root);

% parallel = 0 - no parallel processing, 1 - parallel processing ond
parallel = 0;

if(parallel~= 0)
    if(exist('OCTAVE_VERSION')~=0)
        disp('Octave mode, no parallel processing yet');
    else
        %if nargin < 1
        if(matlabpool('size') == 0) && nargin>=1
            matlabpool close force local
            matlabpool open
            disp('Matlab Multicore processing initialised');
        end
    end
end
clear parallel;