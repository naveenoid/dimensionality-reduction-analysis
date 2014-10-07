function armMotion( xyzSol,t ,opt)
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

pauseInt = 0.1;
%period = 0.003;
period = 0.05;

tInit = 0;

f=figure();
set(f,'DoubleBuffer','on');

h = plot([0,xyzSol(1,1)],[0,xyzSol(1,2)],'r', ...
    [xyzSol(1,1),xyzSol(1,3)],[xyzSol(1,2),xyzSol(1,4)]);

%set(h,'Erasemode','none');

title('motion of the arm');
xlabel('x [m]'); ylabel('y [m]');
axis([-(opt.phys.lengthLink1+opt.phys.lengthLink2) opt.phys.lengthLink1+opt.phys.lengthLink2 ...
       -(opt.phys.lengthLink1+opt.phys.lengthLink2) opt.phys.lengthLink1+opt.phys.lengthLink2]);
axis manual

for i=2:size(xyzSol,1)

    if (t(i) - tInit) >= period;
        set(h(1),'xdata',[0 xyzSol(i,1)],'ydata',[0 xyzSol(i,2)])
        set(h(2),'xdata',[xyzSol(i,1),xyzSol(i,3)],'ydata',[xyzSol(i,2),xyzSol(i,4)])
        tInit = t(i);
        drawnow();
        pause(pauseInt);
        %pause();
    end
end

end

