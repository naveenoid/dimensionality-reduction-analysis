function trajectoryTrace_snapshot_noRobot( xyzSol,t ,opt, fIn,robotPlot, cols)
%ARMMOTION Cartesian motion of the arm's end point. 
%   This function animates the end-point of the arm. 
%   The sequence of positions is described in the matrix xyzSol; the corresponding 
%   timestamps in the vector t. opt contains the description of the robot and other options.
%
%   inputs:
%	xyzSol: Nx2 matrix containing N points in cartesian space (x in the 1st row and y in the 2nd)
%	t: Nx1 vector containing the timestamps correspondning to the points in xyzSol
%	opt: robot options
%
%   Cristiano Alessandro (alessandro@ifi.uzh.ch)
%%

pauseInt = 0;%0.1;
%period = 0.003;
period = 0;%;0.05;

tInit = 0;% 
% h = plot([0,xyzSol(1,1)],[0,xyzSol(1,2)],'r-o', ...
%     [xyzSol(1,1),xyzSol(1,3)],[xyzSol(1,2),xyzSol(1,4)],'r-o','LineWidth',3);
% 
% %h = plot();
% %clf;
% annotatePlotGroup(h,'off');
% %set(h,'Erasemode','none');


f=figure(fIn);
set(f,'DoubleBuffer','on');
% 
% h = plot([0,xyzSol(1,1)],[0,xyzSol(1,2)],'r-o', ...
%     [xyzSol(1,1),xyzSol(1,3)],[xyzSol(1,2),xyzSol(1,4)],'r-o','LineWidth',3);
% 
% %h = plot();
% %clf;
% annotatePlotGroup(h,'off');
% %set(h,'Erasemode','none');

%title('motion of the arm');
xlabel('x [m]'); ylabel('y [m]');
%axis([-(opt.phys.lengthLink1+opt.phys.lengthLink2) opt.phys.lengthLink1+opt.phys.lengthLink2 ...
%       -(opt.phys.lengthLink1+opt.phys.lengthLink2) opt.phys.lengthLink1+opt.phys.lengthLink2]);
%axis manual

frequency = 1;%25;

snapshotStore = zeros(ceil(size(xyzSol,1)/frequency)+1,4);
temp = 1:frequency:size(xyzSol,1);
ctr = 1:size(snapshotStore,1)-1;
snapshotStore(ctr,1:4) = xyzSol(temp,1:4);
%snapshotStore(ctr,1:2) = xyzSol(temp,1:2);
snapshotStore(ctr(end)+1,1:4) = xyzSol(end,1:4);
% if(robotPlot == 0)
%     for i = [2,size(xyzSol,1)]%i=2:25:size(xyzSol,1)
% 
%    % if (t(i) - tInit) >= period;
%     
%      set(h(1),'xdata',[0 xyzSol(i,1)],'ydata',[0 xyzSol(i,2)])
%      set(h(2),'xdata',[xyzSol(i,1),xyzSol(i,3)],'ydata',[xyzSol(i,2),xyzSol(i,4)])
%         
%     %    tInit = t(i);
%         drawnow();
%         hold on;
%     %    pause(pauseInt);
%         %pause();
%   %  end
%     end
% end
%figure(fIn);
if(exist('cols','var')~=0)
    hold on;
 %   p = plot(snapshotStore(:,1),snapshotStore(:,2),'r--');
 %   annotatePlotGroup(p,'off');
    p = plot(snapshotStore(:,3),snapshotStore(:,4),'-','LineWidth',2,'color',cols);
    annotatePlotGroup(p);
  %  p = plot(snapshotStore(end,1),snapshotStore(end,2),'ro','LineWidth',2);
  %  annotatePlotGroup(p,'off');
    p = plot(snapshotStore(end,3),snapshotStore(end,4),'o','LineWidth',2,'color',cols); 
    annotatePlotGroup(p,'off');
  %  axis tight;
  %  axis equal;
else
    hold on;
  %  plot(snapshotStore(:,1),snapshotStore(:,2),'r:');
    plot(snapshotStore(:,3),snapshotStore(:,4),'b:');

  %  plot(snapshotStore(end,1),snapshotStore(end,2),'ro');
    plot(snapshotStore(end,3),snapshotStore(end,4),'bo'); 
 %   axis tight;
 %   axis equal;
end

axis tight;
axis equal;
a = axis;
axis(a+0.01*[-1 1 -1 1]); grid on;
end

