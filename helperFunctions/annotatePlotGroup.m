function [ ] = annotatePlotGroup( p , status)
%ANNOTATEPLOTGROUP Summary of this function goes here
%   Detailed explanation goes here


    pGroup = hggroup;
    set(p,'Parent',pGroup);
    
    if(exist('status','var') ~= 0)
        set(get(get(pGroup,'Annotation'),'LegendInformation'),...
            'IconDisplayStyle','off');                    
    else

        set(get(get(pGroup,'Annotation'),'LegendInformation'),...
            'IconDisplayStyle','on');            
    end
end

