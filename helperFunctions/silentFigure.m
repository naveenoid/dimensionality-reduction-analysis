function [ output_args ] = silentFigure( k )
%SILENTFIGURE Which sets the focus of current figure without stealing focus
%   This avoids matlab figures from stealing focus on linux

    set(0,'CurrentFigure',k);
end

