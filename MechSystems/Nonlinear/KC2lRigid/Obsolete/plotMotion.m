%% Generate trajectory

%Ellipse
axM = 1.9;
ecc = axes2ecc(axM,1.5);
[x,y] = ellipse1(0,0,[axM ecc],0,[0,150]);
%[x,y] = ellipse1(0,0,[axM ecc],-90,[0,150]);

%Parabola
% a = 0.5;
% x = -a*t.^2+1.7;
% y = 2*a*t;
% x = x(1:600); y = y(1:600);

hold on;plot(x,y,'og')

legend('des traj')

%% Plot results
robotData;
iKinSol = iKin([x,y],'up',opt);
fKinSol = fkin(iKinSol(:,1),iKinSol(:,2),opt);
%armMotion(fKinSol,t,opt);
plotResult(fKinSol,t,opt);

legend('des traj','link1','link2')
