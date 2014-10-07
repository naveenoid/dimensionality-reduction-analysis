
xStart = [0;0];
xEnd = [1;1];
tspan = [0 2];
trajectoryType = 4;
opt.kx = 1;
opt.ky = 1;
opt.bx = 0.1;
opt.by = 0.1;

[udes,ydes1,ydes2] = generateBenchmarkTrajectoriesCartesian( xStart, xEnd, tspan,trajectoryType, opt,'r' );

%ydes = @(t)sin25*t
t = linspace(tspan(1),tspan(2),1000);
legendreOpt.order = 5;
legendreOpt.tmax = tspan(end);

legendrefit(ydes1(t), legendreOpt.order);

%W1 = 
[A, Y2, coeff, D] = legendrefit(ydes1(t), legendreOpt.order);


[Psi, p] = legendreBasis(t,legendreOpt);

figure;
plot(t,ydes1(t),'r','LineWidth',2.0); hold on;
%figure;
plot(t,A'*legendreBasis(t,legendreOpt),'--b','LineWidth',2.0);