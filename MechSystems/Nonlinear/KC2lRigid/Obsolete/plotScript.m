
clear errorPID;

%% Calculating

setPoints = generateSetPoints(t'); 
setPoints = setPoints'; 
err = errorPID(x(:,1:2),setPoints,t);

%% Plotting

maxErr = max(max(err(:,1,1),err(:,2,1)));
minErr = min(min(err(:,1,1),err(:,2,1)));
minErr = min(minErr,0);

%Theta 1
figure;
subplot(2,2,1);
plot(t,x(:,1)); hold on;
plot(t,setPoints(:,1),'r');
xlabel('time [s]'); ylabel('Theta 1 [rad]');
legend('x','xdes');

subplot(2,2,3);
plot(t,err(:,1,1)); hold on;
plot(t,zeros(length(t),1),'r');
axis([0 max(t) minErr maxErr]);
xlabel('time [s]'); ylabel('error');

%Theta 2
subplot(2,2,2);
plot(t,x(:,2)); hold on;
plot(t,setPoints(:,2),'r');
xlabel('time [s]'); ylabel('Theta 1 [rad]');
legend('x','xdes');

subplot(2,2,4);
plot(t,err(:,2,1)); hold on;
plot(t,zeros(length(t),1),'r');
axis([0 max(t) minErr maxErr]);
xlabel('time [s]'); ylabel('error');

%% Animation

% xyzSol = fkin( x(:,1), x(:,2), opt );
% 
% pauseInt = 0.2;
% period = 0.1;
% 
% tInit = 0;
% 
% f=figure;
% set(f,'DoubleBuffer','on');
% 
% h = plot([0,xyzSol(1,1)],[0,xyzSol(1,2)],'r', ...
%     [xyzSol(1,1),xyzSol(1,3)],[xyzSol(1,2),xyzSol(1,4)]);
% 
% %set(h,'Erasemode','none');
% 
% title('motion of the arm');
% xlabel('x [m]'); ylabel('y [m]');
% axis([-(opt.lengthLink1+opt.lengthLink2) opt.lengthLink1+opt.lengthLink2 ...
%        -(opt.lengthLink1+opt.lengthLink2) opt.lengthLink1+opt.lengthLink2]);
% axis manual
% 
% for i=2:size(xyzSol,1)
% 
%     if (t(i) - tInit) >= period;
%         set(h(1),'xdata',[0 xyzSol(i,1)],'ydata',[0 xyzSol(i,2)])
%         set(h(2),'xdata',[xyzSol(i,1),xyzSol(i,3)],'ydata',[xyzSol(i,2),xyzSol(i,4)])
%         tInit = t(i);
%         drawnow();
%         pause(pauseInt);
%     end
% end