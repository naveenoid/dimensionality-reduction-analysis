
% sample call: 
% [t, y] = CreateInteractiveSpline(sampleCount, [0 tau], [-10 10], [0 anchors(dim); tau anchors(dim)], [0 0]);

function [ X, Y, SplineStruct, XKnots, YKnots ] = ...
    CreateInteractiveSpline( SampleCount, XLim, YLim, PreDefinedPoints, EndSlopes )
% CREATEINTERACTIVESPLINE Function to provide a spline from fitting points
% chosen interactively by the user by clicking in a window

figInput = figure;
xlim(XLim); ylim(YLim);
button = 0;
X = []; Y = [];
XKnots = []; YKnots = []; 

if(~isempty(PreDefinedPoints))
    XKnots = [XKnots; PreDefinedPoints(:,1)];
    YKnots = [YKnots; PreDefinedPoints(:,2)];
    [temp, sortIdx] = sort(XKnots, 'ascend');
    if(length(XKnots)>1)
        if(isempty(EndSlopes))
            SplineStruct = spline(XKnots(sortIdx), YKnots(sortIdx));
        else
            SplineStruct = spline(XKnots(sortIdx), [EndSlopes(1); YKnots(sortIdx); EndSlopes(2)]);
        end    
        X = linspace(XKnots(sortIdx(1)), XKnots(sortIdx(end)), SampleCount);
        Y = ppval(SplineStruct, X);
        plot(X, Y, 'k', 'LineWidth', 3);
        hold on;
    end
    scatter(XKnots, YKnots, 'ro');
    xlim(XLim); ylim(YLim);
    hold off;
    drawnow;
end

while(button~=3)
    [x, y, button] = ginput(1);
    XKnots = [XKnots; x];
    YKnots = [YKnots; y];
    [temp, sortIdx] = sort(XKnots, 'ascend');
    if(length(XKnots)>1)
        if(isempty(EndSlopes))
            SplineStruct = spline(XKnots(sortIdx), YKnots(sortIdx));
        else
            SplineStruct = spline(XKnots(sortIdx), [EndSlopes(1); YKnots(sortIdx); EndSlopes(2)]);
        end
        X = linspace(XKnots(sortIdx(1)), XKnots(sortIdx(end)), SampleCount);
        Y = ppval(SplineStruct, X);
        plot(X, Y, 'k', 'LineWidth', 3);
        hold on;
    end
    scatter(XKnots, YKnots, 'ro');
    xlim(XLim); ylim(YLim);
    hold off;
    drawnow;
end

pause(1);
close(figInput);

end

