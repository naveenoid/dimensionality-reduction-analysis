function err = errorPID(x,xDes,t)
%% Computation of the PID errors
%   x: state at time t
%   xDes: desired state
%   t: time
%
% Cristiano Alessandro (alessandro@ifi.uzh.ch)
%

persistent epOld eiOld edOld tOld;


%epNew = xDes - x;
ep = xDes - x;

if isempty(epOld)
    %ep = epNew;
    ed = 0*ones(size(xDes,1),size(xDes,2));
    %ei = epNew*ones(size(xDes,1),size(xDes,2));
    %ei = epNew;
    ei = ep;
    
    edOld = ed;
    eiOld = ei;
    epOld = ep;
else
    dt = t - tOld;
    if dt == 0
        %ed =  0*ones(size(xDes,1),size(xDes,2));
        ed = edOld;
        ei = eiOld;
        ep = epOld;
    else
        %ed = (epNew - ep)/dt;
        %ep = epNew;
        ed = (ep - epOld)/dt;
        ei = ep*dt + eiOld;
        
        edOld = ed;
        eiOld = ei;
        epOld = ep;
    end
    
    %ei = ep*dt + eiOld;
    %ep = epNew;
end

%edOld = ed;
%eiOld = ei;
tOld = t;

err = zeros(size(ed,1),size(ed,2),3);
err(:,:,1) = ep;
err(:,:,2) = ed;
err(:,:,3) = ei;