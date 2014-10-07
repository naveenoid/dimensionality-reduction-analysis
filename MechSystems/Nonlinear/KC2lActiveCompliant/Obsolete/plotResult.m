function plotResult( xyzSol,t ,opt, xyzTraj)

xTot = 0.53;
yTot = 0.46;

%pauseInt = 0.01;
%period = 0.003;
period = 0.07;

tInit = 0;

f=figure;
%set(f,'DoubleBuffer','on');

h = plot([0,xyzSol(1,1)],[0,xyzSol(1,2)],'r', ...
    [xyzSol(1,1),xyzSol(1,3)],[xyzSol(1,2),xyzSol(1,4)]);

%set(h,'Erasemode','none');
hold on;

minX = min([0; xyzSol(:,1); xyzSol(:,3)]);
maxX = max([0; xyzSol(:,1); xyzSol(:,3)]);
minY = min([0; xyzSol(:,2); xyzSol(:,4)]);
maxY = max([0; xyzSol(:,2); xyzSol(:,4)]);

spanX = maxX-minX;
spanY = maxY-minY;

diffX = xTot-spanX;
diffY = yTot-spanY;

mgn = 0.05;

title('Motion of the Arm');
xlabel('x [m]'); ylabel('y [m]');
% axis([-(opt.phys.lengthLink1+opt.phys.lengthLink2) opt.phys.lengthLink1+opt.phys.lengthLink2 ...
%        -(opt.phys.lengthLink1+opt.phys.lengthLink2) opt.phys.lengthLink1+opt.phys.lengthLink2]);

%axis manual
axis equal
axis([minX-diffX/2-mgn maxX+diffX/2+mgn minY-diffY/2-mgn maxY+diffY/2+mgn]);

for i=2:size(xyzSol,1)

    if (t(i) - tInit) >= period;
        
        plot([0,xyzSol(i,1)],[0,xyzSol(i,2)],'r', ...
            [xyzSol(i,1),xyzSol(i,3)],[xyzSol(i,2),xyzSol(i,4)]);
        
        tInit = t(i);
        %pause(pauseInt);
    end
end
grid on;

plot(xyzTraj(:,3),xyzTraj(:,4),'k')

plot2svg('./fig/curl.svg', f);

end